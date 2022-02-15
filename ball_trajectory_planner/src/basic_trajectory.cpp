#include <cstdio>
#include <rclcpp/rclcpp.hpp>

struct KinematicSoln {

};

class BasicTrajectory : public rclcpp::Node {
  public:
    BasicTrajectory() : Node("basic_trajectory"){

    }


  private:


};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world ball_trajectory_planner package\n");
  return 0;
}