CONTIKI_PROJECT=er-server
all: $(CONTIKI_PROJECT)

CONTIKI=../..

MODULES += core/net/mac/tsch

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

# define rest resource file
REST_RESOURCES_DIR = ./resources
REST_RESOURCES_FILES = $(notdir $(shell find $(REST_RESOURCES_DIR) -name '*.c'))

# add to project
PROJECTDIRS += $(REST_RESOURCES_DIR)
PROJECT_SOURCEFILES += $(REST_RESOURCES_FILES)

# add observe
# PROJECT_SOURCEFILES += er-observe-client.c

# REST Engine shall use Erbium CoAP implementation
APPS += er-coap
APPS += rest-engine
APPS += orchestra

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
