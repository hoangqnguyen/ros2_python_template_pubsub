# Package and Node
PACKAGE ?= py_pubsub
NODE ?= talker

# Environment
SHELL := /bin/bash
ENV ?= /opt/ros/*/setup.bash

# Commands
.PHONY: all
all: clean build run

.PHONY: build
build:
	source $(ENV); \
	colcon build

.PHONY: run
run:
	source install/setup.bash; \
	ros2 run $(PACKAGE) $(NODE)

.PHONY: clean
clean:
	rm -rf build log install

.PHONY: echo
echo:
	ros2 topic echo /topic