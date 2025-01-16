#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

HOMEKIT_PATH ?= $(abspath $(shell pwd)/components)
COMMON_COMPONENT_PATH ?= $(abspath $(shell pwd)/components/common)

PROJECT_NAME := hap-gateway
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/
EXTRA_COMPONENT_DIRS += $(HOMEKIT_PATH)/homekit
EXTRA_COMPONENT_DIRS += $(COMMON_COMPONENT_PATH)

include $(IDF_PATH)/make/project.mk

