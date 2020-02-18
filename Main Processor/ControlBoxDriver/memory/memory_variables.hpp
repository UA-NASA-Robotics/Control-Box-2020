#ifndef MEMORY_VARIABLES_HPP
#define MEMORY_VARIABLES_HPP

/*--------------------------------- Page One ---------------------------------*/

// Variables for Communications
#define ROUTER_ADDRESS    0x00
#define MACRO_TYPE      0x01
#define MACRO_DATA      0x02
#define REQUEST       0x03
#define CONNECTED     0x04
#define TIMEOUT_IN_PROGRESS 0x07
#define TRANSMIT_MACRO    0x08
#define MACRO_IN_PROGRESS 0x09
#define CONNECTED_TIME_ELAP 0x0A

/*--------------------------------- Page Two ---------------------------------*/

// Analog Input Request Flags

#define JOYSTICK_LEFT_X   0x20
#define JOYSTICK_LEFT_Y   0x21
#define JOYSTICK_RIGHT_X  0x22
#define JOYSTICK_RIGHT_Y  0x23
#define SLIDER_LEFT     0x24
#define SLIDER_RIGHT    0x25

/*-------------------------------- Page Three --------------------------------*/

// Arcade Button Request Flags

#define ARCADE_BUTTON_0_FLAG 0x30 // Emergency Stop
#define ARCADE_BUTTON_1_FLAG 0x31 // Plow Up
#define ARCADE_BUTTON_2_FLAG 0x32 // Plow Down
#define ARCADE_BUTTON_3_FLAG 0x33 // Arm Up
#define ARCADE_BUTTON_4_FLAG 0x34 // Bucket Down
#define ARCADE_BUTTON_5_FLAG 0x35 // Bucket Up
#define ARCADE_BUTTON_6_FLAG 0x36 // Arm Down
#define ARCADE_BUTTON_7_FLAG 0x37 // Plow Up
#define ARCADE_BUTTON_8_FLAG 0x38 // Plow Down

/*-------------------------------- Page Four ---------------------------------*/

// Push Button Request Flags

const uint8_t PAGE_FOUR = 0x40;

#define PUSH_BUTTON_MACRO_PENDING  2

#define PUSH_BUTTON_0_FLAG 0x40
#define PUSH_BUTTON_1_FLAG 0x41
#define PUSH_BUTTON_2_FLAG 0x42
#define PUSH_BUTTON_3_FLAG 0x43
#define PUSH_BUTTON_4_FLAG 0x44
#define PUSH_BUTTON_5_FLAG 0x45
#define PUSH_BUTTON_6_FLAG 0x46
#define PUSH_BUTTON_7_FLAG 0x47
#define PUSH_BUTTON_8_FLAG 0x48
#define PUSH_BUTTON_9_FLAG 0x49
#define PUSH_BUTTON_10_FLAG 0x4A
#define PUSH_BUTTON_11_FLAG 0x4B
#define PUSH_BUTTON_12_FLAG 0x4C
#define PUSH_BUTTON_13_FLAG 0x4D
#define PUSH_BUTTON_14_FLAG 0x4E
#define PUSH_BUTTON_15_FLAG 0x4F

/*-------------------------------- Page Four ---------------------------------*/

#define FORCE_SENSOR_FEEDBACK 0x50

#endif // MEMORY_VARIABLES_HPP
