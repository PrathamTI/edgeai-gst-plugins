#!/bin/bash
set -e

echo " =================================== Compiling modules =================================== "

cd ~
cd /opt/edgeai-tiovx-modules/build 

make -j2 
make install 

echo " =================================== Compiling Plugins ==================================="

cd ~
cd /opt/edgeai-gst-plugins
ninja -C build 
ninja -C build install 

 
echo " =================================== Executing the tiovxfc pipeline =================================== "

echo " =================================== FC raw pipeline is =================================== "

echo " GST_DEBUG=tiovxfc:7 gst-launch-1.0 \
  v4l2src device=/dev/video-imx219-cam0 io-mode=dmabuf ! \
  video/x-bayer,format=rggb,width=1920,height=1080 ! \
  tiovxfcvissmsc \
    sensor-name="SENSOR_SONY_IMX219_RPI" \
    dcc-fc-isp-file="/opt/imaging/imx219/linear/dcc_viss_1920x1080.bin" \
    interpolation-method=bilinear \
    bypass-cac=true \
    bypass-dwb=true \
    bypass-nsf4=false \
    ee-mode=EE_MODE_OFF \
    name=fc \
  fc.src_0 ! \
  videoconvert ! \
  video/x-raw,format=NV12,width=1280,height=720 ! \
  kmssink driver-name=tidss plane-properties="s,zpos=1" "


GST_DEBUG=tiovxfc:7 gst-launch-1.0 \
  v4l2src device=/dev/video-imx219-cam0 io-mode=dmabuf ! \
  video/x-bayer,format=rggb,width=1920,height=1080 ! \
  tiovxfcvissmsc \
    sensor-name="SENSOR_SONY_IMX219_RPI" \
    dcc-fc-isp-file="/opt/imaging/imx219/linear/dcc_viss_1920x1080.bin" \
    interpolation-method=bilinear \
    bypass-cac=true \
    bypass-dwb=true \
    bypass-nsf4=false \
    ee-mode=EE_MODE_OFF \
    name=fc \
  fc.src_0 ! \
  videoconvert ! \
  video/x-raw,format=NV12,width=1280,height=720 ! \
  kmssink driver-name=tidss plane-properties=s,zpos=1"  "


#gst-launch-1.0 v4l2src io-mode=dmabuf-import device=/dev/video-imx219-cam0 ! video/x-bayer,width=1920,height=1080,format=rggb ! tiovxisp sensor-name=SENSOR_SONY_IMX219_RPI dcc-isp-file=/opt/imaging/imx219/linear/dcc_viss_1920x1080.bin sink_0::dcc-2a-file=/opt/imaging/imx219/linear/dcc_2a_1920x1080.bin sink_0::device=/dev/v4l-imx219-subdev0 ! video/x-raw,format=NV12 ! queue ! kmssink driver-name=tidss plane-properties=s,zpos=1



# ========================= ISP Plugin check ========================= 

# GST_DEBUG=tiovxviss:7 gst-launch-1.0 v4l2src io-mode=dmabuf-import device=/dev/video-imx219-cam0 ! video/x-bayer,width=1920,height=1080,format=rggb ! tiovxisp sensor-name=SENSOR_SONY_IMX219_RPI dcc-isp-file=/opt/imaging/imx219/linear/dcc_viss_1920x1080.bin sink_0::dcc-2a-file=/opt/imaging/imx219/linear/dcc_2a_1920x1080.bin sink_0::device=/dev/v4l-imx219-subdev0 ! video/x-raw,format=NV12 ! queue ! kmssink driver-name=tidss plane-properties=s,zpos=1

# ========================= MSC Plugin check ========================= 

# GST_DEBUG=tiovxmultiscaler:7 gst-launch-1.0   videotestsrc is-live=true num-buffers=5 ! "video/x-raw,format=NV12,width=1280,height=720" !   tiovxmultiscaler name=multi multi. ! "video/x-raw,format=NV12,width=640,height=480" ! queue ! fakesink silent=false -v -e  