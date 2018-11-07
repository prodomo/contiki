CONTIKI_PROJECT = node
all: $(CONTIKI_PROJECT)

CONTIKI=../../..

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
MODULES += core/net/mac/tsch

CONTIKI_WITH_IPV6 = 1

APPS += er-coap
APPS += rest-engine
APPS += orchestra

# define rest resource file
REST_RESOURCES_DIR = ./resources
REST_RESOURCES_FILES = $(notdir $(shell find $(REST_RESOURCES_DIR) -name '*.c'))

# add to project
PROJECTDIRS += $(REST_RESOURCES_DIR)
PROJECT_SOURCEFILES += $(REST_RESOURCES_FILES)

# add observe
# PROJECT_SOURCEFILES += er-observe-client.c

include $(CONTIKI)/Makefile.include
