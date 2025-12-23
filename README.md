# lidar_perception
LiDAR Perception & Obstacle Avoidance in ROS 2

### Executing code
1. **Launch CARLA**:
   ```bash
   ./CarlaUE4.sh -prefernvidia -quality-level=Low -carla-server -benchmark -fps=15 -windowed -ResX=800 -ResY=600
   ```

2. **Run Carla sensor Node**:
   ```bash
   git clone https://github.com/thuvaragak/carla_project.git
   cd carla_project
   colcon build
   . install/setup.bash
   ros2 run carla_vehicle carla_sensor
   ```

3. **Run Lidar Perception Node**:
   ```bash
   ros2 run obstacle_avoidance lasersensor
   ```

4. **Visualization**:
   ```bash
   Rviz2
   ```
   Add  /obstacle_bboxes topic to visualize obstacles