CUDA_VER ?= $(shell basename $$(readlink -f /usr/local/cuda) | sed 's/cuda-//')
STREAMING_DIR := streaming
STREAMING_BIN := $(STREAMING_DIR)/csi_h264_rtsp_server
STREAMING_OBJ := $(STREAMING_DIR)/csi_h264_rtsp_server.o
LEGACY_STREAMING_ARTIFACTS := csi_h264_rtsp_server csi_h264_rtsp_server.o

APPS := deepstream_yolo26_rtsp_target_control $(STREAMING_BIN)

TARGET_DEVICE = $(shell gcc -dumpmachine | cut -f1 -d -)
NVDS_VERSION ?= $(shell basename $$(readlink -f /opt/nvidia/deepstream/deepstream) | sed 's/deepstream-//')
LIB_INSTALL_DIR ?= /opt/nvidia/deepstream/deepstream-$(NVDS_VERSION)/lib/

ifeq ($(TARGET_DEVICE),aarch64)
  CFLAGS += -DPLATFORM_TEGRA
endif

SRCS := deepstream_yolo26_rtsp_target_control.c $(STREAMING_DIR)/csi_h264_rtsp_server.c
OBJS := deepstream_yolo26_rtsp_target_control.o $(STREAMING_OBJ)

PKGS := gstreamer-1.0 gstreamer-video-1.0 gstreamer-rtsp-1.0 gstreamer-rtsp-server-1.0 glib-2.0

CFLAGS += -I/opt/nvidia/deepstream/deepstream-$(NVDS_VERSION)/sources/includes
CFLAGS += -I/usr/local/cuda-$(CUDA_VER)/include
CFLAGS += $(shell pkg-config --cflags $(PKGS))
CFLAGS += -Wall -Wextra -O2

LIBS := $(shell pkg-config --libs $(PKGS))
LIBS += -L/usr/local/cuda-$(CUDA_VER)/lib64/ -lcudart
LIBS += -L$(LIB_INSTALL_DIR) -lnvdsgst_meta -lnvds_meta -lnvds_yml_parser -lcuda
LIBS += -lm
LIBS += -Wl,-rpath,$(LIB_INSTALL_DIR)

all: $(APPS)

%.o: %.c Makefile
	$(CC) -c -o $@ $(CFLAGS) $<

deepstream_yolo26_rtsp_target_control: deepstream_yolo26_rtsp_target_control.o Makefile
	$(CC) -o $@ $< $(LIBS)

csi_h264_rtsp_server: $(STREAMING_BIN)

$(STREAMING_BIN): $(STREAMING_OBJ) Makefile
	$(CC) -o $@ $< $(LIBS)

clean:
	rm -f $(APPS) $(OBJS) $(LEGACY_STREAMING_ARTIFACTS)
