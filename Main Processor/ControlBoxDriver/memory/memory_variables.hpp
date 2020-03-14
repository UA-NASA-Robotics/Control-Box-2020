#ifndef MEMORY_VARIABLES_HPP
#define MEMORY_VARIABLES_HPP

/*--------------------------------- Page One ---------------------------------*/

// Variables for Communications
#define ROUTER_ADDRESS      0x00
#define CONNECTED_TIME_ELAP     0x01
#define MACRO_DATA        0x02
#define REQUEST         0x03
#define CONNECTED       0x04
#define TIMEOUT_IN_PROGRESS   0x07
#define MACRO_TYPE        0x08
#define MACRO_IN_PROGRESS   0x09
#define X_coord         0x0A
#define Y_coord         0x0B
#define HeadingVal        0x0C

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
#define ARCADE_E_STOP_FLAG   0x39

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
#define ROBOT_X_LOCATION  0x55
#define ROBOT_Y_LOCATION  0x56
#define ROBOT_HEADING   0x57


#define MAIN_MOTOR_CELL_1 0x60
#define MAIN_MOTOR_CELL_2 0x61
#define MAIN_MOTOR_CELL_3 0x63
#define MAIN_MOTOR_CELL_4 0x64
#define MAIN_MOTOR_CELL_5 0x65
#define MAIN_MOTOR_CELL_6 0x66
#define MAIN_MOTOR_CELL_7 0x67
#define MAIN_MOTOR_CELL_8 0x68
#define MAIN_EE_CELL_1    0x69
#define MAIN_EE_CELL_2    0x6A

#define SECOND_MOTOR_CELL_1 0x6B
#define SECOND_MOTOR_CELL_2 0x6C
#define SECOND_MOTOR_CELL_3 0x6D
#define SECOND_MOTOR_CELL_4 0x6E
#define SECOND_MOTOR_CELL_5 0x6F
#define SECOND_MOTOR_CELL_6 0x70
#define SECOND_MOTOR_CELL_7 0x71
#define SECOND_MOTOR_CELL_8 0x72
#define SECOND_EE_CELL_1  0x73
#define SECOND_EE_CELL_2  0x74

#endif // MEMORY_VARIABLES_HPP
