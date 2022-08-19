echo "Loading ROS environment variables..."
export PYTHONPATH=""
source packages/install/setup.sh
export PYTHONPATH=$PYTHONPATH:$(pwd)/packages/install/lib
export GAZEBO_MODEL_PATH=$(pwd)/packages:$(pwd)/packages/truck_gazebo/worlds
export GAZEBO_MASTER_URI=http://localhost:9090
