target-obj:=yolov3_demo
TARGET?=yolov3_demo
SHELL:=/bin/bash
CURRENT_DIR:=$(shell pwd)
OUTPUT_BIN:=yolov3_demo
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
	cp ./data/video.mp4 $(OUTPUT_DIR)/
	cp scripts/param.conf $(OUTPUT_DIR)/
	# cp $(OUTPUT_DIR)/$(OUTPUT_BIN) $(CURRENT_DIR)/;
