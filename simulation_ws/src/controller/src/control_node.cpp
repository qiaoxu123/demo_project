#include "controller/controller.h"

using namespace control;

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_node");
    control::Control Controller;
    Controller.run();
    return 0;
}