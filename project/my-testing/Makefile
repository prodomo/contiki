DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: mqtt-demo

CONTIKI_WITH_IPV6 = 1

APPS += mqtt
# APPS += collect-view
APPS += orchestra
MODULES += core/net/mac/tsch

ifeq ($(MAKE_WITH_ORCHESTRA),1)
CFLAGS += -DWITH_ORCHESTRA=1
endif

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
