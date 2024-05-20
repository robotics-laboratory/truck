.SILENT:
.ONESHELL:

SHELL := /bin/bash

.PHONY: all
all:
	$(error Please use explicit targets)

.PHONY: build
# packages="first_pkg second_pkg third_pkg..."
build:
	source ${ROS_ROOT}/setup.sh
	colcon --log-base /dev/null build \
		--base-paths packages \
		--executor parallel \
		--parallel-workers $$(nproc) \
		--symlink-install \
		--packages-up-to $(packages)

.PHONY: build-all
build-all:
	source ${ROS_ROOT}/setup.sh
	colcon --log-base /dev/null build \
		--base-paths packages \
		--executor parallel \
		--parallel-workers $$(nproc) \
		--symlink-install

.PHONY: test
# packages="first_pkg second_pkg third_pkg..."
test:
	source ${ROS_ROOT}/setup.sh
	source install/setup.sh
	colcon --log-base /dev/null test \
		--base-paths packages \
		--return-code-on-test-failure \
		--executor parallel \
		--parallel-workers $$(nproc) \
		--event-handlers console_cohesion+ \
		--packages-select $(packages)

.PHONY: test-all
test-all:
	source ${ROS_ROOT}/setup.sh
	source install/setup.sh
	colcon --log-base /dev/null test \
		--base-paths packages \
		--return-code-on-test-failure \
		--executor parallel \
		--parallel-workers $$(nproc) \
		--event-handlers console_cohesion+

.PHONY: clean
clean:
	rm -rf build install
