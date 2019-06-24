# velma_sim_gazebo {#velma_sim_gazebo_readme}
This package defines and implements a group of simulated real effectors of Velma core agent.
It is Gazebo plugin and Orocos components for WUT Velma Robot simulation in Gazebo.

It provides ROS launch file:
 * velma_gazebo_re.launch - runs the subsystem and simulator; it uses unpause_on_init script to unpause
the simulation after initialization procedure is complete; arguments:
   * *gui* - run also Gazebo client
   * *headless* - run Gazebo without graphic card support (all rendering is disabled, even for visual sensors)
   * *verbose*
   * *debug* - if `true`, the deployer is executed in gdb.
   * *world_name* - world file name with respect to GAZEBO_RESOURCE_PATH
   * *load_robot_description* - if `true`, the launch file loads robot description from urdf and runs robot_state_publisher
   * *GAZEBO_MODEL_PATH* - please refer to Gazebo documentation
   * *GAZEBO_RESOURCE_PATH* - please refer to Gazebo documentation
   * *use_kinect* - `true` or `false`: enable or disable kinect simulation
   * *ORO_LOGLEVEL* - orocos log level, the higher number the more verbose output
   * *profile* - physics profile as defined in world file


