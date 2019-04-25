
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := librtp
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := RTP (Real-time Transport Protocol) library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DRTP_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/rtcp_pkt.c \
	src/rtp_jitter.c \
	src/rtp_pkt.c
LOCAL_LIBRARIES := \
	libfutils \
	libpomp \
	libulog

ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

ifdef TARGET_TEST
include $(CLEAR_VARS)
LOCAL_MODULE := tst-rtp
LOCAL_CFLAGS := -std=gnu99
LOCAL_SRC_FILES := tests/test_rtp_ntp.c
LOCAL_LIBRARIES := librtp
include $(BUILD_EXECUTABLE)
endif
