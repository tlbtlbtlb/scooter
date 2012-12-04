

default: Park.highbw.mpg Park.lowbw.mpg

AVIFILES=Park.avi

include $(HOME)/libtlb/mkvideo.inc

install: Park.highbw.mpg Park.lowbw.mpg
	install -c Park.highbw.mpg $(HOME)/website/ur/robot-video.highbw.mpg
	install -c Park.lowbw.mpg $(HOME)/website/ur/robot-video.lowbw.mpg
