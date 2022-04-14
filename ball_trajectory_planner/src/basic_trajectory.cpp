#include <fmt/format.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <map>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/motor_msg.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int8.hpp>

#define GRAV 9.81

using std::placeholders::_1;

/**
 * @brief data struct for holding a kinematic solution to the problem before
 * it becomes a joint state
 * 
 */
struct KinematicSoln {
    double range;
    double ballVel;
    double elevAngle;
    double azmuithAngle;
    double timeDelta;
};

class BasicTrajectory : public rclcpp::Node {
 private:
    // subscriber for reciever position
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr playerPoseSub;

    // subscriber for last robot pose
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robotPoseSub;

    // subscriber to cause a reset of the node
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resetSub;

    // timer to create a new trajectory
    rclcpp::TimerBase::SharedPtr updateTimer; 

    // publishers of joint information
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr jointValid;
    std_msgs::msg::Int8 validMsg;

    // range lookup table for wheel velocity bands
    // input is range in m (rounded to 0.5 m increments)
    std::map<double, double> rangeLUT = {};
    double lutStep = 0.5, wheelRatio;
    
    // solver parameters 
    double maxSolve, solveStep, solvePeriod;
    int minSamples;

    // predictor parameters
    double predBufferMaxLen = 10;

	std::deque<geometry_msgs::msg::Pose::SharedPtr> relPoseBuffer;

 public:
    BasicTrajectory() : Node("basic_trajectory") {
        RCLCPP_DEBUG(get_logger(), "Initializing parameters");

        declare_parameter("topic.player_pose", "player/pose");

        declare_parameter("range_lut.step", 0.5);
        declare_parameter("range_lut.values", std::vector<double>{0.0, 0.5});
        declare_parameter("range_lut.ratio", 0.2794 * 3);

        declare_parameter("solver.run_period", 0.25);
        declare_parameter("solver.min_pose_samples", 1);

        declare_parameter("predict.max_buffer_len", 300);


        RCLCPP_DEBUG(get_logger(), "Initializing subscribers");
        playerPoseSub = this->create_subscription<geometry_msgs::msg::Pose>(get_parameter("topic.player_pose").as_string(), rclcpp::SensorDataQoS(), std::bind(&BasicTrajectory::newPlayerPoseData, this, _1));
        resetSub = this->create_subscription<std_msgs::msg::Empty>("/sys/reset", rclcpp::SystemDefaultsQoS(), std::bind(&BasicTrajectory::handleReset, this, _1));

        RCLCPP_DEBUG(get_logger(), "Initializing solver params");

        solvePeriod = get_parameter("solver.run_period").as_double();
        minSamples = get_parameter("solver.min_pose_samples").as_int();
        
        RCLCPP_DEBUG(get_logger(), "Initializing timers");

        updateTimer = this->create_wall_timer(std::chrono::duration<double>(solvePeriod), std::bind(&BasicTrajectory::buildOptimTraj, this));      
        updateTimer->cancel();  
        
        RCLCPP_DEBUG(get_logger(), "Initializing publishers");

        jointPub = this->create_publisher<sensor_msgs::msg::JointState>("trajectory/desired_state", rclcpp::SystemDefaultsQoS());
        jointValid = this->create_publisher<std_msgs::msg::Int8>("trajectory/valid", rclcpp::SystemDefaultsQoS());
        validMsg = std_msgs::msg::Int8();

        RCLCPP_DEBUG(get_logger(), "Initializing range lut params");

        // init LUT based on step increments
        wheelRatio = get_parameter("range_lut.ratio").as_double();
        lutStep = get_parameter("range_lut.step").as_double();

        auto lutInit = get_parameter("range_lut.values").as_double_array();
        for (int i = 0; i < lutInit.size(); i++) {
            rangeLUT.insert({i, lutInit.at(i)});
        }

        RCLCPP_INFO(get_logger(), "Using LUT step value of %f m / value with max range of %f m", lutStep, lutStep * rangeLUT.size());

        RCLCPP_DEBUG(get_logger(), "Initializing predictor params");

        predBufferMaxLen = get_parameter("predict.max_buffer_len").as_int();

        RCLCPP_INFO(get_logger(), "Trajectory node initalized");
    }

    /**
     * @brief callback to handle all reset code for this node
     * TODO check if more things need to be reset, but the pose buffer and update timer should be the only thing thus far
     */
    void handleReset(const std_msgs::msg::Empty::SharedPtr){
        relPoseBuffer.clear();
        updateTimer->cancel();
    }

	/**
	 * @brief Get the range to the target based off of its relative position
	 * 
	 * @param relPose the pose of the player relative to the robot
	 * @return double the range to the target in m
	 */
    double getRange(const geometry_msgs::msg::Pose::SharedPtr relPose) {
        return std::sqrt(relPose->position.x * relPose->position.x + relPose->position.y * relPose->position.y);
    }

	/**
	 * @brief Get the azmuith angle needed to hit a desired relative position
	 * 
	 * @param relPose relative position in m
	 * @return double the angle in radians to the pan motor
	 */
    double getAzmuithAngle(const geometry_msgs::msg::Pose::SharedPtr relPose) {
        return std::atan2(relPose->position.y, relPose->position.x);
    }

	/**
	 * @brief Get the elevation angle of the tilt system for a given ball velocity and relative
	 * player position
	 * 
	 * @param relPose relative position of the player
	 * @param ballVel velocity of the ball leaving the system in m/s
	 * @return double elevation angle in radians
	 */
    double getElevationAngle(const geometry_msgs::msg::Pose::SharedPtr relPose, double ballVel, double range) {
        double speedSq = ballVel * ballVel;
        double speedQu = std::pow(ballVel, 4);
        double root = speedQu - GRAV * (GRAV * range * range + 2 * speedSq * relPose->position.z);

        // The minus sign here should give the lower solution (less than 45)
		return std::atan2(speedSq - std::sqrt(root), (GRAV * range));
	}

	/**
	 * @brief calcluate the time of flight of the ball in seconds based on relative position of the target,
	 * ball velocity, and elevation angle
	 * 
	 * @param zDelta height delta of the player
	 * @param elevAngle elevation angle of the system
	 * @param ballVel velocity the ball will leave the system at
	 * @return double time of ball flight in seconds
	 */
    double getTof(const geometry_msgs::msg::Pose::SharedPtr relPose, double elevAngle, double ballVel) {
        return (ballVel * std::sin(elevAngle)) / GRAV + std::sqrt(ballVel * ballVel * std::sin(elevAngle) + 2 * GRAV * relPose->position.z) / GRAV;
    }

    /**
     * @brief uses the LUT to convert lateral range to angular velocity of the wheel system
     *
     * @param range the range of the shot in meters
     * @return double the output velocity of the flywheels in m/s
     */
    double lookupBallVel(double range) {
        int step = (int)(range / lutStep);
        if(step > rangeLUT.size()){
            RCLCPP_ERROR(get_logger(), "Range outside of Lookup table, got value: %i with LUT size: %i for range %f", step, rangeLUT.size(), range);
            return rangeLUT.at(rangeLUT.size() - 1);
        }
        return rangeLUT.at(step);
    }

    double linearVeltoAngVel(double linear){
        return linear / wheelRatio;
    }

    KinematicSoln calcToTarget(const geometry_msgs::msg::Pose::SharedPtr newPose) {
        auto soln = KinematicSoln();
        soln.range = getRange(newPose);
        soln.ballVel = lookupBallVel(soln.range);
        soln.azmuithAngle = getAzmuithAngle(newPose);
        // have to negate the elevation angle to force the robot to tilt backwards
        soln.elevAngle = -getElevationAngle(newPose, soln.ballVel, soln.range);
        soln.timeDelta = getTof(newPose, soln.elevAngle, soln.ballVel);
        
        return soln;
    }

	/**
	 * @brief recieve new information about the player position
	 * 
	 * @param newPose 
	 */
	void newPlayerPoseData(const geometry_msgs::msg::Pose::SharedPtr newPose) {
        relPoseBuffer.push_back(newPose);

        if(relPoseBuffer.size() > predBufferMaxLen){
            relPoseBuffer.pop_front();
        }

        if(relPoseBuffer.size() < minSamples){
            RCLCPP_DEBUG(get_logger(), "Pose buffer size too small to build trajectories: %i", relPoseBuffer.size());
        } else if (updateTimer->is_canceled()){
            RCLCPP_INFO(get_logger(), "Pose buffer size ready to build trajectories: %i", relPoseBuffer.size());
            updateTimer->reset();
        }
    }

    void buildOptimTraj(){
        RCLCPP_DEBUG(get_logger(), "rebuilding Trajectory");

        //seed predicted pose with current pose to generate first possible intercept
        auto predictedPose = relPoseBuffer.at(0);

        // compute the intial solution
        auto solution = calcToTarget(predictedPose);
        // Check if trajectory was generated okay
        if(std::isnan(solution.elevAngle)){
            // only print on first invalid
            if(validMsg.data != -1) 
            RCLCPP_ERROR(get_logger(), "No valid trajectory exists with given point [%f, %f, %f] and veloicty limits",
                 predictedPose->position.x, predictedPose->position.y, predictedPose->position.z);

            validMsg.data = -1;
            jointValid->publish(validMsg);
            return;
        } else if (validMsg.data == -1){
            RCLCPP_WARN(get_logger(), "Found valid trajectory with given point [%f, %f, %f]", 
                predictedPose->position.x, predictedPose->position.y, predictedPose->position.z);
        }

        validMsg.data = 0;
        jointValid->publish(validMsg);

        // Continue if trajectory is valid
        RCLCPP_DEBUG(get_logger(), "First intercept time: %f", solution.timeDelta);

        // RCLCPP_INFO(get_logger(), "Got valid soln, moving robot!");

        auto angWheelVel = linearVeltoAngVel(solution.ballVel);

        auto jointState = sensor_msgs::msg::JointState();

        jointState.name = {"pan_motor", "tilt_motor", "left_wheel_motor", "right_wheel_motor"};
        jointState.position = {solution.azmuithAngle, solution.elevAngle, 0.0, 0.0};
        jointState.velocity = {0.0, 0.0, angWheelVel, angWheelVel};
        jointState.effort = {0.0, 0.0, 0.0, 0.0};

        jointPub->publish(jointState);
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BasicTrajectory>();

    rclcpp::spin(node);

    rclcpp::shutdown();
}