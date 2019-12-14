int main(int argc, char **argv) {
    ros::init(argc, argv, "simulation_core");
    simulation::SimulationCore simulation_core;
    simulation_core.run();
}