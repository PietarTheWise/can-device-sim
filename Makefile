CXX ?= clang++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -Wpedantic -Iinclude

BUILD_DIR := build

MAIN_SRC := src/main.cpp src/can_bus.cpp src/device.cpp src/robotic_arm.cpp src/conveyor_belt.cpp src/vision_sensor.cpp src/controller.cpp src/random_event.cpp
MAIN_BIN := $(BUILD_DIR)/main

RING_TEST_SRC := tests/ring_buffer_test.cpp
RING_TEST_BIN := $(BUILD_DIR)/ring_buffer_test

CAN_BUS_TEST_SRC := tests/can_bus_test.cpp src/device.cpp src/can_bus.cpp
CAN_BUS_TEST_BIN := $(BUILD_DIR)/can_bus_test

VISION_TEST_SRC := tests/vision_sensor_test.cpp src/device.cpp src/vision_sensor.cpp src/random_event.cpp
VISION_TEST_BIN := $(BUILD_DIR)/vision_sensor_test

CONTROLLER_SAFETY_TEST_SRC := tests/controller_safety_test.cpp src/device.cpp src/controller.cpp
CONTROLLER_SAFETY_TEST_BIN := $(BUILD_DIR)/controller_safety_test

.PHONY: all main test run-tests clean

all: main

main: $(MAIN_BIN)

test: $(RING_TEST_BIN) $(CAN_BUS_TEST_BIN) $(VISION_TEST_BIN) $(CONTROLLER_SAFETY_TEST_BIN)

run-tests: test
	./$(RING_TEST_BIN)
	./$(CAN_BUS_TEST_BIN)
	./$(VISION_TEST_BIN)
	./$(CONTROLLER_SAFETY_TEST_BIN)

$(MAIN_BIN): $(MAIN_SRC) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(MAIN_SRC) -o $(MAIN_BIN)

$(RING_TEST_BIN): $(RING_TEST_SRC) include/ring_buffer.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(RING_TEST_SRC) -o $(RING_TEST_BIN)

$(CAN_BUS_TEST_BIN): $(CAN_BUS_TEST_SRC) include/can_bus.hpp include/device.hpp include/can_frame.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(CAN_BUS_TEST_SRC) -o $(CAN_BUS_TEST_BIN)

$(VISION_TEST_BIN): $(VISION_TEST_SRC) include/vision_sensor.hpp include/device.hpp include/can_frame.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(VISION_TEST_SRC) -o $(VISION_TEST_BIN)

$(CONTROLLER_SAFETY_TEST_BIN): $(CONTROLLER_SAFETY_TEST_SRC) include/controller.hpp include/device.hpp include/can_frame.hpp include/can_protocol.hpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(CONTROLLER_SAFETY_TEST_SRC) -o $(CONTROLLER_SAFETY_TEST_BIN)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
