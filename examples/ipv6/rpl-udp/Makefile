all: udp-client udp-server
APPS=servreg-hack
CONTIKI=../../..

WITH_UIP6=1
UIP_CONF_IPV6=1
CFLAGS+=-DSERVER_REPLY

PROJECT_SOURCEFILES += i2c-drv.c

include $(CONTIKI)/Makefile.include
