# Android.mk
#
# Copyright (C) 2013 Sony Mobile Communications AB.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2, as
# published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

LOCAL_PATH:= $(call my-dir)

#
# mount.ecryptfs command
#

include $(CLEAR_VARS)
LOCAL_SRC_FILES:= ecryptfs.c mount.ecryptfs.c
LOCAL_C_INCLUDES := external/openssl/include
LOCAL_SHARED_LIBRARIES := libcutils libcrypto
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:= mount.ecryptfs
include $(BUILD_EXECUTABLE)

#
# umount.ecryptfs command
#

include $(CLEAR_VARS)
LOCAL_SRC_FILES:= umount.ecryptfs.c
LOCAL_SHARED_LIBRARIES := libcutils
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE:= umount.ecryptfs
include $(BUILD_EXECUTABLE)
