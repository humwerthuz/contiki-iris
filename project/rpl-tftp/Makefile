all: border-router

CONTIKI=../..

WITH_UIP6=1
UIP_CONF_IPV6=1
TFTPD_CONF_EXEC=1
UIP_CONF_TCP=0
SYMBOLS=1

#TARGET = iris
#APPS += webserver
APPS += tftpd

COFFEE_FILES=4
#SYMBOLS=1
#CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\" -DUIP_FALLBACK_INTERFACE=rpl_interface -DCOFFEE_FILES=3 -DCOFFEE_ADDRESS=0x14000 -DWITH_EXEC=1
#DEFINES=PROJECT_CONF_H="project-conf.h",UIP_FALLBACK_INTERFACE=rpl_interface,COFFEE_FILES=3,COFFEE_ADDRESS=0x14000,WITH_EXEC=1
DEFINES=PROJECT_CONF_H=\"project-conf.h\",UIP_FALLBACK_INTERFACE=rpl_interface
#PROJECT_SOURCEFILES += slip-bridge.c httpd-simple.c
PROJECT_SOURCEFILES += slip-bridge.c
COFFEE_ADDRESS=0x18000

include $(CONTIKI)/Makefile.include

$(CONTIKI)/tools/tunslip6:	$(CONTIKI)/tools/tunslip6.c
	(cd $(CONTIKI)/tools && $(MAKE) tunslip6)

connect-router:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 aaaa::1/64 -s /dev/ttyUSB1

connect-router-cooja:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 -a 127.0.0.1 aaaa::1/64

%.tftp-upload:%
	busybox tftp $(IPV6ADDR) -p -l $* -b 176
%.tftp-download:
	busybox tftp $(IPV6ADDR) -g -r $* -b 176
