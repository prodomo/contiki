# CONTIKI_PROJECT = udp-sink
CONTIKI_PROJECT = udp-sender udp-sink

all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES += collect-common.c

CONTIKI=../..
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

CONTIKI_WITH_IPV6 = 1

# MAKE_WITH_ORCHESTRA ?= 0 # force Orchestra from command line
# MAKE_WITH_SECURITY ?= 0 # force Security from command line
APPS += powertrace collect-view
APPS += orchestra
MODULES += core/net/mac/tsch
 # CFLAGS += -ffunction-sections
 # LDFLAGS += -Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__
ifeq ($(MAKE_WITH_ORCHESTRA),1)
CFLAGS += -DWITH_ORCHESTRA=1
endif

ifeq ($(MAKE_WITH_SECURITY),1)
CFLAGS += -DWITH_SECURITY=1
endif

include $(CONTIKI)/Makefile.include
