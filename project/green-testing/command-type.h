#ifndef COMMAND_TYPE_H_
#define COMMAND_TYPE_H_

#define UNKNOWN_TYPE 0
#define RATE_TYPE 1
#define BEEP_TYPE 2

struct command_msg {
  int8_t type;
  int8_t value;
};

#endif