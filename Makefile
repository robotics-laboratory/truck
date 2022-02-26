.PHONY: all build_planner build_unwrapper build start stop

all:
	echo "Please, use explicit targets"

build_planner:
	colcon build --packages-up-to planning_node

build_unwrapper:
	colcon build --packages-up-to unwrapping_node

build: build_planner build_unwrapper

start: stop
	./scripts/start.sh

pub:
	./scripts/pub.sh

stop:
	kill -- -"$$(cat log/group.pid)" || true
