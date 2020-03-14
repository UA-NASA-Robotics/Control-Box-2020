#ifndef SCREEN_OBJECTS_HPP
#define SCREEN_OBJECTS_HPP

#define NO_OBJECT 0

#define Form             0x0A
#define Image            0x0C
#define Winbutton        0x06
#define LEDDigits        0x0F
#define Statictext       0x15
#define Led              0x0E
#define Gauge            0x0B

enum BattryCells {
	Main_Motor_Cell_1= 1,
	Main_Motor_Cell_2,
	Main_Motor_Cell_3,
	Main_Motor_Cell_4,
	Main_Motor_Cell_8,
	Main_Motor_Cell_7,
	Main_Motor_Cell_6,
	Main_Motor_Cell_5,
	Main_EE_Cell_1,
	Main_EE_Cell_2,
	Second_Motor_Cell_1,
	Second_Motor_Cell_2,
	Second_Motor_Cell_3,
	Second_Motor_Cell_4,
	Second_Motor_Cell_5,
	Second_Motor_Cell_6,
	Second_Motor_Cell_7,
	Second_Motor_Cell_8,
	Second_EE_Cell_1,
	Second_EE_Cell_2,
	Main_E_bat,
	Dig_E_bat,
	Main_Motor_Bat,
	Dig_Motor_Bat,
	X_LOC,
	Y_LOC,
	HEADING


};
#define CONNECTED_LED 0
#define MACRO_RUNNING 1



#endif // SCREEN_OBJECTS_HPP