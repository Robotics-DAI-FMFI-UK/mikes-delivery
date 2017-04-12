CC=gcc
CPP=g++
PROG=mikes
SRCS=mikes.c \
     public_relations.c \
     mikes_logs.c \
     navigation.c \
     gui.c \
     windows.c \
     config/config.c \
     config_mikes.c \
     base_module.c \
     util.c \
     ncurses_control.c 
CPPSRCS=lidar.cpp
OBJS=${SRCS:.c=.o}
CPPOBJS=${CPPSRCS:.cpp=.o}
CFLAGS=-std=c11 -D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include/ -g -Wall
CPPFLAGS=-D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include/ -g -Wall -Wno-write-strings
LDFLAGS=-lpthread -lrt -lcairo -lX11 -lm -lncurses -L/usr/local/rplidar/sdk/output/Linux/Release -lrplidar_sdk
PREFIX=/usr/local

all: ${OBJS} ${CPPOBJS}
	${CPP} ${OBJS} ${CPPOBJS} -o ${PROG} ${CFLAGS} ${LDFLAGS}

install:
	mv ${PROG} ${PREFIX}/bin
	cp mikes.sh /etc/init.d/mikes
	chmod a+x /etc/init.d/mikes
	update-rc.d mikes defaults
	cp config_mikes.cfg ${PREFIX}/etc

uninstall:
	update-rc.d -f mikes remove
	rm -f ${PREFIX}/bin/${PROG}

clean:
	rm -f *.o ${PROG}
