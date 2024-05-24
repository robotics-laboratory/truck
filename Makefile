.SILENT:
.ONESHELL:

SHELL = /bin/bash

CMAKE_EXPORT_COMPILE_COMMANDS ?= ON
CMAKE_BUILD_TYPE ?= Release
CMAKE_TOOLS_ADDRESS_SANITIZER ?= OFF

CXXFLAGS := \
    ${CXXFLAGS} \
    $(shell if [ "${CMAKE_TOOLS_ADDRESS_SANITIZER}" = "ON" ]; then echo "-fsanitize=address"; fi)

CMAKE_ARGS ?= \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=${CMAKE_EXPORT_COMPILE_COMMANDS} \
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    -DCMAKE_CXX_FLAGS="${CXXFLAGS}"

FILES_TO_LINT := $(shell find . \( -name "*.h" -or -name "*.cpp" -or -name "*.cc" \) \
                    -not -path "*/build/*" -not -path "*/install/*" -not -path "*/log/*")

CLANG_TIDY_FIX_FILES ?= OFF
CLANG_TIDY_FLAGS := \
	$(shell if [ "${CLANG_TIDY_FIX_FILES}" = "ON" ]; then echo "-fix"; fi)

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
		--cmake-args ${CMAKE_ARGS} \
		--packages-up-to $(packages)

.PHONY: build-all
build-all:
	source ${ROS_ROOT}/setup.sh
	colcon --log-base /dev/null build \
		--base-paths packages \
		--executor parallel \
		--parallel-workers $$(nproc) \
		--symlink-install \
		--cmake-args ${CMAKE_ARGS}

.PHONY: test
# packages="first_pkg_name second_pkg_name third_pkg_name"
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

.PHONY: lint
# packages="first_pkg_name second_pkg_name third_pkg_name"
lint:
	run-clang-tidy-${CLANG_VERSION} -p=build ${CLANG_TIDY_FLAGS} $(packages)

.PHONY: lint-all
lint-all:
	run-clang-tidy-${CLANG_VERSION} -p=build ${CLANG_TIDY_FLAGS} ${FILES_TO_LINT}

.PHONY: clean
clean:
	rm -rf build install
