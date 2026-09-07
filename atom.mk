
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
LOCAL_MODULE := tst-librtp
LOCAL_CFLAGS := -std=gnu99 -DTARGET_TEST -D_GNU_SOURCE
LOCAL_C_INCLUDES := $(LOCAL_PATH)/src
LOCAL_SRC_FILES := \
	tests/rtp_test.c \
	tests/rtp_test_compare.c \
	tests/rtp_test_monkey.c \
	tests/rtp_test_ntp.c \
	tests/rtp_test_priv.c \
	tests/rtp_test_rtcp_pkt.c \
	tests/rtp_test_rtp_jitter.c \
	tests/rtp_test_rtp_pkt.c \
	tests/rtp_test_timestamp.c
LOCAL_LIBRARIES := \
	libcunit \
	libfutils \
	libpomp \
	librtp
include $(BUILD_EXECUTABLE)
endif
