#!/bin/bash
cd "$( dirname "${BASH_SOURCE[0]}" )/.." 
echo "[ VTOL mission START signal ]"

docker compose exec vtol_mission bash -c "
  source /ros2_ws/install/setup.bash &&
  ros2 topic pub /mission/start std_msgs/msg/Empty '{}' -1
"

echo "start signal sent."