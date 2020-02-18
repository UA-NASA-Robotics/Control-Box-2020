#include "request.hpp"

int get_push_button_index (const Request & request)
{
  int index = -1;
  for (int i = 0; i < NUM_PUSH_BUTTONS; ++i)
    if (PUSH_BUTTON_REQUESTS[i] == request)
      index = i;
  return index;
}