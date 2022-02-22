#include <fmt/format.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/motor_msg.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define GRAV 9.81

using std::placeholders::_1;

struct KinematicSoln {
};

class BasicTrajectory : public rclcpp::Node {
 private:
    // subscriber for reciever position
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr playerPoseSub;

    // subscriber for last robot pose
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robotPoseSub;

    // publishers of joint information
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPub;
    rclcpp::Publisher<can_msgs::msg::MotorMsg>::SharedPtr leftWheel, rightWheel, panMotor, tiltMotor;

    // range lookup table for wheel velocity bands
    // input is range in m (rounded to 0.5 m increments)
    std::map<double, double> rangeLUT = {};
    double lutStep = 0.5;

	std::vector<geometry_msgs::msg::Pose> relPoseBuffer;

    sensor_msgs::msg::JointState::SharedPtr lastRobotPose;

 public:
    std::string getMotorTopic(const std::string& name) {
        return fmt::format(get_parameter("topic.motor_template").as_string(), get_parameter("name." + name).as_string());
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
        declare_parameter("range_lut.values", {});

        RCLCPP_DEBUG(get_logger(), "Initializing player pose and robot joint state subscribers");
        playerPoseSub = this->create_subscription<geometry_msgs::msg::Pose>(get_parameter("topic.player_pose").as_string(), rclcpp::SensorDataQoS(), std::bind(&BasicTrajectory::newPlayerPoseData, this, _1));
        robotPoseSub = this->create_subscription<sensor_msgs::msg::JointState>(get_parameter("topic.robot_pose").as_string(), rclcpp::SensorDataQoS(), std::bind(&BasicTrajectory::newRobotPoseData, this, _1));

        RCLCPP_DEBUG(get_logger(), "Initializing publishers");

        jointPub = this->create_publisher<sensor_msgs::msg::JointState>("trajectory/desired_state", rclcpp::SystemDefaultsQoS());
        leftWheel = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("left_wheel_motor"), rclcpp::SystemDefaultsQoS());
        rightWheel = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("right_wheel_motor"), rclcpp::SystemDefaultsQoS());
        panMotor = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("pan_motor"), rclcpp::SystemDefaultsQoS());
        tiltMotor = this->create_publisher<can_msgs::msg::MotorMsg>(getMotorTopic("tilt_motor"), rclcpp::SystemDefaultsQoS());

        RCLCPP_DEBUG(get_logger(), "Initializing range lut");

        // init LUT based on half meter increments
        auto lutInit = get_parameter("range_lut.values").as_double_array();
        auto lutStep = 1.0 / get_parameter("range_lut.step").as_double();
        for (int i = 0; i < lutInit.size(); i++) {
            // convert table units from rpm to rad/s
            rangeLUT.insert({i, lutInit.at(i) / (2 * M_PI) * 60.0});
        }

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
    double getElevationAngle(const geometry_msgs::msg::Pose::SharedPtr relPose, double ballVel) {
		auto range = getRange(relPose);
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
    double calcTof(double zDelta, double elevAngle, double ballVel) {
        return (ballVel * std::sin(elevAngle)) / GRAV + std::sqrt(ballVel * ballVel * std::sin(elevAngle) + 2 * GRAV * zDelta) / GRAV;
    }

    /**
     * @brief uses the LUT to convert lateral range to angular velocity of the wheel system
     *
     * @param range the range of the shot in meters
     * @return double the output velocity of the flywheels in rad/s
     */
    double lookupAngVel(double range) {
        int step = (int)(range / lutStep);
        return rangeLUT.at(step);
    }

	/**
	 * @brief extrapolate the players postion into the futrure by specified time tDelta
	 * 
	 * @param tDelta 
	 * @param currentPose 
	 * @return geometry_msgs::msg::Pose::SharedPtr 
	 */
	geometry_msgs::msg::Pose::SharedPtr predictPlayerPose(double tDelta, const geometry_msgs::msg::Pose::SharedPtr currentPose){

	}

	/**
	 * @brief recieve new information about the player position
	 * 
	 * @param newPose 
	 */
	void newPlayerPoseData(const geometry_msgs::msg::Pose::SharedPtr newPose) {
        RCLCPP_DEBUG(get_logger(), "Got new relative player location");


    }

    void calcToTarget() {

    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BasicTrajectory>();

    rclcpp::spin(node);

    rclcpp::shutdown();
}