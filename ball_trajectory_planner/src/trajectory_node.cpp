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
    rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr leftWheel, rightWheel, panMotor, tiltMotor;

    // range lookup table for wheel velocity bands
    // input is range in m (rounded to 0.5 m increments)
    std::map<double, double> rangeLUT = {};
    double rangeLutStep = 0.5, wheelRatio;
    
    // solver parameters 
    double maxSolve, solveStep, solvePeriod;
    int minSamples;

    // predictor parameters
    double predBufferMaxLen = 10;

	std::deque<geometry_msgs::msg::Pose::SharedPtr> relPoseBuffer;

    sensor_msgs::msg::JointState::SharedPtr lastRobotPose;

 public:
    std::string getMotorTopic(const std::string& name) {
        return fmt::format(get_parameter("topic.motor_template").as_string(), get_parameter("names." + name).as_string());
    }

    BasicTrajectory() : Node("basic_trajectory") {
        RCLCPP_DEBUG(get_logger(), "Initializing parameters");

        declare_parameter("names.left_wheel_motor", "left_wheel_motor");
        declare_parameter("names.right_wheel_motor", "right_wheel_motor");
        declare_parameter("names.pan_motor", "pan_motor");
        declare_parameter("names.tilt_motor", "tilt_motor");

        declare_parameter("topic.motor_template", "motor/{}/demand");
        declare_parameter("topic.player_pose", "player/pose");
        declare_parameter("topic.robot_pose", "motor/joint_state");

        declare_parameter("range_lut.step", 0.5);
        declare_parameter("range_lut.values", std::vector<double>{0.0, 0.5});
        declare_parameter("range_lut.ratio", 0.2794 * 3);

        declare_parameter("solver.max_time", 5.0);
        declare_parameter("solver.time_step", 0.2);
        declare_parameter("solver.run_period", 0.25);
        declare_parameter("solver.min_pose_samples", 10);

        declare_parameter("predict.max_buffer_len", 300);


        RCLCPP_DEBUG(get_logger(), "Initializing subscribers");
        playerPoseSub = this->create_subscription<geometry_msgs::msg::Pose>(get_parameter("topic.player_pose").as_string(), rclcpp::SensorDataQoS(), std::bind(&BasicTrajectory::newPlayerPoseData, this, _1));
        robotPoseSub = this->create_subscription<sensor_msgs::msg::JointState>(get_parameter("topic.robot_pose").as_string(), rclcpp::SensorDataQoS(), std::bind(&BasicTrajectory::newRobotPoseData, this, _1));
        resetSub = this->create_subscription<std_msgs::msg::Empty>("/sys/reset", rclcpp::SystemDefaultsQoS(), std::bind(&BasicTrajectory::handleReset, this, _1));

        RCLCPP_DEBUG(get_logger(), "Initializing solver params");

        maxSolve = get_parameter("solver.max_time").as_double();
        solveStep = get_parameter("solver.time_step").as_double();
        solvePeriod = get_parameter("solver.run_period").as_double();
        minSamples = get_parameter("solver.min_pose_samples").as_int();
        
        RCLCPP_DEBUG(get_logger(), "Initializing timers");

        updateTimer = this->create_wall_timer(std::chrono::duration<double>(solvePeriod), std::bind(&BasicTrajectory::buildOptimTraj, this));      
        updateTimer->cancel();  
        
        RCLCPP_DEBUG(get_logger(), "Initializing publishers");

        jointPub = this->create_publisher<sensor_msgs::msg::JointState>("trajectory/desired_state", rclcpp::SystemDefaultsQoS());
        leftWheel = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("left_wheel_motor"), rclcpp::SystemDefaultsQoS());
        rightWheel = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("right_wheel_motor"), rclcpp::SystemDefaultsQoS());
        panMotor = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("pan_motor"), rclcpp::SystemDefaultsQoS());
        tiltMotor = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("tilt_motor"), rclcpp::SystemDefaultsQoS());

        RCLCPP_DEBUG(get_logger(), "Initializing range lut params");

        // init LUT based on step increments
        wheelRatio = get_parameter("range_lut.ratio").as_double();
        rangeLutStep = 1.0 / get_parameter("range_lut.step").as_double();

        auto lutInit = get_parameter("range_lut.values").as_double_array();
        for (int i = 0; i < lutInit.size(); i++) {
            rangeLUT.insert({i, lutInit.at(i)});
        }

        RCLCPP_DEBUG(get_logger(), "Initializing predictor params");

        predBufferMaxLen = get_parameter("predict.max_buffer_len").as_int();

        RCLCPP_INFO(get_logger(), "Trajectory node initalized");
    }

	/**
	 * @brief callback for the robot joint position information
	 * 
	 * @param newPose the current position of the robot joints
	 */
	void newRobotPoseData(const sensor_msgs::msg::JointState::SharedPtr newPose) {
        lastRobotPose = newPose;
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
		return std::atan(ballVel * ballVel - std::sqrt(std::pow(ballVel, 4) - GRAV * (GRAV * range * range + 2 * ballVel * ballVel * relPose->position.z) / (GRAV * range)));
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
        int step = (int)(range / rangeLutStep);
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
        soln.elevAngle = getElevationAngle(newPose, soln.ballVel, soln.range);
        soln.timeDelta = getTof(newPose, soln.elevAngle, soln.ballVel);
        
        return soln;
    }

	/**
	 * @brief extrapolate the players postion into the futrure by specified time tDelta
	 * 
	 * @param tDelta 
	 * @param currentPose 
	 * @return geometry_msgs::msg::Pose::SharedPtr 
	 */
	geometry_msgs::msg::Pose::SharedPtr predictPlayerPose(double tDelta, const geometry_msgs::msg::Pose::SharedPtr currentPose){
        //TODO not sure how to do this part yet...
        
        // could either linearly interp fwd in time

        // or could try to fit a trajectory to the path of the user

        // maybe both, depending on large t vs small t
        
        // NOTE FOR NOW THIS IS JUST A PASS THROUGH TO TEST THE OPTIM ALGORITHM
        return currentPose;
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

        RCLCPP_DEBUG(get_logger(), "First intercept time: %f", solution.timeDelta);

        // start at t = timedelta as it is the closest possible solution
        // computing anything earlier than this would produce waste
        double tPredict = solution.timeDelta;

        // true while the intercept point has not yet been found
        bool noInt = true;

        // make sure looop has 2 exlpicit exit conditions for finding a solution, and for timing out
        while(noInt && tPredict <= maxSolve){

            // bump the solve step to the next increment
            tPredict += solveStep;

            // guess the players new position based on the old, the local buffer, and the time to extrapolate to
            predictedPose = predictPlayerPose(tPredict, predictedPose);
            RCLCPP_DEBUG(get_logger(), "Player predicted %fs to (X: %f, Y: %f)", tPredict, predictedPose->position.x, predictedPose->position.y);

            // find the new solution
            solution = calcToTarget(predictedPose);  

            RCLCPP_DEBUG(get_logger(), "New intercept time: %f", solution.timeDelta);

            // evaluate exit condition. algo 
            noInt = solution.timeDelta > tPredict;          
            
        }
        if(noInt){
            RCLCPP_ERROR(get_logger(), "Could not find solution to (X: %f, Y: %f) within time delta of %f", 
                predictedPose->position.x, predictedPose->position.y, maxSolve);
        } else {
            // got a valid solution here, lets send it to the machine
            RCLCPP_INFO(get_logger(), "Got valid soln, moving robot!");

            auto angWheelVel = linearVeltoAngVel(solution.ballVel);

            auto panMsg = can_msgs::msg::MotorMsg();
            panMsg.control_mode = 1;
            panMsg.demand = solution.azmuithAngle;
            panMotor->publish(panMsg);

            auto tiltMsg = can_msgs::msg::MotorMsg();
            tiltMsg.control_mode = 1;
            tiltMsg.demand = solution.azmuithAngle;
            tiltMotor->publish(tiltMsg);

            auto wheelMsg = can_msgs::msg::MotorMsg();
            tiltMsg.control_mode = 2;
            tiltMsg.demand = angWheelVel;
            leftWheel->publish(wheelMsg);
            rightWheel->publish(wheelMsg);

        }
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BasicTrajectory>();

    rclcpp::spin(node);

    rclcpp::shutdown();
}
