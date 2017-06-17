# Copyright (C) 2013 Sony Mobile Communications AB.
# All rights, including trade secret rights, reserved.
#
# @author PLD/Security United

#
# Tests for eCryptfs mount and umount commands
#

ifneq ($(TARGET_BUILD_VARIANT),user)
  LOCAL_PATH := $(call my-dir)

  include $(CLEAR_VARS)

  LOCAL_SRC_FILES := test_ecryptfs.sh
  LOCAL_MODULE := test_ecryptfs.sh
  LOCAL_MODULE_CLASS := EXECUTABLES
  LOCAL_MODULE_TAGS := eng
  LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)

  include $(BUILD_PREBUILT)
endif #TARGET_BUILD_VARIANT

