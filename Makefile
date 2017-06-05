CC=gcc
CPP=g++
PROG=mikes
SRCS=mikes.c \
     public_relations.c \
     mikes_logs.c \
     steering.c \
     gui.c \
     windows.c \
     planner.c \
     mcl.c \
     config/config.c \
     config_mikes.c \
     base_module.c \
     util.c \
     ncurses_control.c \
     pq.c \
     astar.c \
     pose.c \
     ride.c
OPTIMIZE=-O3
CPPSRCS=lidar.cpp
OBJS=${SRCS:.c=.o}
CPPOBJS=${CPPSRCS:.cpp=.o}
CFLAGS=${OPTIMIZE} -std=c11 -D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include -I/usr/include/librsvg-2.0/librsvg -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I/usr/include/libxml2 -I/usr/include/gdk-pixbuf-2.0 -Wall
CPPFLAGS=${OPTIMIZE} -D_BSD_SOURCE -D_XOPEN_SOURCE=600 -I/usr/include/cairo -I/usr/local/rplidar/sdk/sdk/include -Wall -Wno-write-strings -I/usr/include/librsvg-2.0/librsvg -I/usr/include/glib-2.0 -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include -I/usr/include/gdk-pixbuf-2.0 
LDFLAGS=-lpthread -lrt -lcairo -lX11 -lm -lncurses -L/usr/local/rplidar/sdk/output/Linux/Release -lrplidar_sdk -lrsvg-2 -lxml2
PREFIX=/usr/local

all: ${OBJS} ${CPPOBJS} 
	${CPP} ${OBJS} ${CPPOBJS} -o ${PROG} ${CFLAGS} ${LDFLAGS}

install:
	mv ${PROG} ${PREFIX}/bin
	cp mikes.sh /etc/init.d/mikes
	chmod a+x /etc/init.d/mikes
	update-rc.d mikes defaults
	cp config_mikes.cfg ${PREFIX}/etc

test:	test_pq.c pq.* test_astar.c
	${CC} -o test_pq test_pq.c pq.c -g ${CFLAGS} ${LDFLAGS}
	${CC} -o test_astar test_astar.c astar.c pq.c -g ${CFLAGS} ${LDFLAGS}
	${CC} -o test_pose test_pose.c pose.c mikes_logs.c config/config.c config_mikes.c -g ${CFLAGS} ${LDFLAGS}
     

uninstall:
	update-rc.d -f mikes remove
	rm -f ${PREFIX}/bin/${PROG}

clean:
	rm -f *.o ${PROG}
