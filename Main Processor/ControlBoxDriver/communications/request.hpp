#ifndef REQUEST_HPP
#define REQUEST_HPP

/*--------------------------- Request Definitions ----------------------------*/

typedef enum
{
  NO_REQUEST,
  
  // Arcade Buttons
  EMERGENCY_STOP,
  BUCKET_UP,
  BUCKET_DOWN,
  ARM_UP,
  ARM_DOWN,
  PLOW_UP,
  PLOW_DOWN,
  
  // Joystick
  JOYSTICK,
  
  // Push Buttons
  MACRO_0,
  MACRO_1,
  MACRO_2,
  MACRO_3,
  MACRO_4,
  MACRO_5,
  MACRO_6,
  MACRO_7,
  MACRO_8,
  MACRO_9,
  MACRO_10,
  MACRO_11,
  MACRO_12,
  MACRO_13,
  MACRO_14,
  MACRO_15
} Request;

/*--------------------------- Push Button Requests ---------------------------*/

const int NUM_PUSH_BUTTONS = 16;

const Request PUSH_BUTTON_REQUESTS [NUM_PUSH_BUTTONS] =
{
  MACRO_0,
  MACRO_1,
  MACRO_2,
  MACRO_3,
  MACRO_4,
  MACRO_5,
  MACRO_6,
  MACRO_7,
  MACRO_8,
  MACRO_9,
  MACRO_10,
  MACRO_11,
  MACRO_12,
  MACRO_13,
  MACRO_14,
  MACRO_15
};

int get_push_button_index (const Request &);

/*-------------------------- Arcade Button Requests --------------------------*/

const int NUM_ARCADE_BUTTONS = 9;

const Request ARCADE_BUTTON_REQUESTS [NUM_ARCADE_BUTTONS] =
{
  EMERGENCY_STOP,
  PLOW_UP,
  PLOW_DOWN,
  ARM_UP,
  BUCKET_DOWN,
  BUCKET_UP,
  ARM_DOWN,
  PLOW_UP,
  PLOW_DOWN
};

#endif // REQUEST_HPP
