MODULE_NAME = mechdng
MODULE_OBJS = $(BUILD_DIR)/mechdng.o
TOP_DIR = ../..

# include modules environment
include $(TOP_DIR)/modules/Makefile
.DEFAULT_GOAL := $(BUILD_DIR)/module_complete
