.PHONY: all build start stop

all:
	echo "Please, use explicit targets"

build:
	colcon build --packages-up-to truck

launch:
	. install/setup.sh && ros2 launch truck planner.yaml

pub:
	./scripts/pub.sh
