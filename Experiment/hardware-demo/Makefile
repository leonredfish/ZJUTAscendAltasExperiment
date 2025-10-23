
target-obj = hardware_demo
TARGET?=hardware_demo
SHELL:=/bin/bash
CURRENT_DIR := $(shell pwd)
OUTPUT_BIN = hardware_demo
BUILD_DIR ?=build
OUTPUT_DIR ?=out
SRC_DIR ?=src
APP_JOBS =4

all:
	rm -rf $(BUILD_DIR) $(OUTPUT_DIR); 
	rm -rf $(TARGET);
	mkdir -p $(BUILD_DIR);
	mkdir -p $(OUTPUT_DIR);
	pushd  $(BUILD_DIR)/; \
		cmake $(CURRENT_DIR)/$(SRC_DIR); \
		make -j$(APP_JOBS); \
	popd ; \
	cp $(OUTPUT_DIR)/$(OUTPUT_BIN) $(CURRENT_DIR)/;