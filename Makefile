.PHONY: all cmake-settings setup build-all build-one build-clean
.SILENT:
.ONESHELL:

SHELL := /bin/bash

all:
	echo "Please, use explicit targets" >&2

cmake-settings:
	echo "Exporting cmake settings..." >&2
	export CMAKE_BUILD_PARALLEL_LEVEL=$$(($$(nproc) - 1))

setup:
	echo "Loading environment variables..." >&2
	export PYTHONPATH=$$PYTHONPATH:$$(pwd)/packages/install/model/lib
	source packages/install/setup.sh

build-all: cmake-settings
	cd packages
	colcon build --merge-install --packages-skip-up-to $(exclude)

build-one: cmake-settings
	cd packages
	colcon build --merge-install --packages-up-to $(package)

build-clean:
	cd packages
	rm -rf build install log
