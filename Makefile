.PHONY: all build-all build-one build-up build-clean
.SILENT:
.ONESHELL:

SHELL := /bin/bash

all:
	echo "Please, use explicit targets" >&2

build-all:
	cd packages
	colcon build --merge-install --packages-skip-up-to $(skip)

build-one:
	cd packages
	colcon build --merge-install --packages-select $(package)

build-up:
	cd packages
	colcon build --merge-install --packages-up-to $(package)

build-clean:
	cd packages
	rm -rf build install log
