
// ---------------------------------------------------------------------------
// Created by Francisco Malpartida on 20/08/11.
// Copyright (C) - 2018
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License v3.0
//    along with this program.
//    If not, see <https://www.gnu.org/licenses/gpl-3.0.en.html>.
//
// ---------------------------------------------------------------------------
//
// Thread Safe: No
// Extendable: Yes
//
// @file I2CIO.h
// This file implements a basic IO library using the PCF8574 I2C IO Expander
// chip.
//
// @brief
// Implement a basic IO library to drive the PCF8574* I2C IO Expander ASIC.
// The library implements basic IO general methods to configure IO pin direction
// read and write uint8_t operations and basic pin level routines to set or read
// a particular IO port.
//
//
// @version API 1.0.0
//
// @author F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------



#include "i2c.hpp"


#include <inttypes.h>
#include <stdio.h>
#include "../uart/uart_3.hpp"
#include "I2CIO.hpp"
#define  OUTPUT 1
#define INPUT 0


// CLASS VARIABLES
// ---------------------------------------------------------------------------


// CONSTRUCTOR
// ---------------------------------------------------------------------------
I2CIO::I2CIO ( )
{
	_i2cAddr     = 0x0;
	_dirMask     = 0xFF;    // mark all as INPUTs
	_shadow      = 0x0;     // no values set
	_initialised = false;
}

// PUBLIC METHODS
// ---------------------------------------------------------------------------


//
// begin
I2C Wire;
int I2CIO::begin (  uint8_t i2cAddr )
{
	_i2cAddr = i2cAddr;

	Wire.set_address (_i2cAddr);

	//_initialised = isAvailable ( _i2cAddr );

	_shadow = Wire.read (&_shadow,1); // Remove the byte read don't need it.

	return ( 1 );
}

//
// pinMode
void I2CIO::pinMode ( uint8_t pin, uint8_t dir )
{
//  if ( _initialised )
//  {
	if ( OUTPUT == dir )
	{
		_dirMask &= ~( 1 << pin );
	}
	else
	{
		_dirMask |= ( 1 << pin );
	}
//	}
}

//
// portMode
void I2CIO::portMode ( uint8_t dir )
{

//  if ( _initialised )
//  {
	if ( dir == INPUT )
	{
		_dirMask = 0xFF;
	}
	else
	{
		_dirMask = 0x00;
	}
	//}
}

//
// read
uint8_t I2CIO::read ( void )
{
	uint8_t retVal = 0;

//  if ( _initialised )
//  {
	uint8_t receivedVal;
	Wire.read (&receivedVal,1 );
	retVal = ( _dirMask &  receivedVal);

//	}
	return ( retVal );
}

//
// write
int I2CIO::write ( uint8_t value )
{
	int status = 0;

//  if ( _initialised )
//  {
	// Only write HIGH the values of the ports that have been initialised as
	// outputs updating the output shadow of the device

	//
	// 15-FEB-2018 - fix, all I/Os initialized as input must be written as HIGH
	//    _shadow = ( value & ~(_dirMask) );
	_shadow = ( value | _dirMask );


	Wire.write ( &_shadow,1 );

//	}
	return ( (status == 0) );
}

//
// digitalRead
uint8_t I2CIO::digitalRead ( uint8_t pin )
{
	uint8_t pinVal = 0;

	// Check if initialised and that the pin is within range of the device
	// -------------------------------------------------------------------
//  if ( ( _initialised ) && ( pin <= 7 ) )
//  {
	// Remove the values which are not inputs and get the value of the pin
	pinVal = this->read() & _dirMask;
	pinVal = ( pinVal >> pin ) & 0x01; // Get the pin value
	//}
	return (pinVal);
}

//
// digitalWrite
int I2CIO::digitalWrite ( uint8_t pin, uint8_t level )
{
	uint8_t writeVal;
	int status = 0;

	// Check if initialised and that the pin is within range of the device
	// -------------------------------------------------------------------
//  if ( ( _initialised ) && ( pin <= 7 ) )
//  {
	// Only write to HIGH the port if the port has been configured as
	// an OUTPUT pin. Add the new state of the pin to the shadow
	writeVal = ( 1 << pin ) & ~_dirMask;

	if ( level == 1 )
	{
		_shadow |= writeVal;

	}
	else
	{
		_shadow &= ~writeVal;
	}
	status = this->write ( _shadow );
//	}
	return ( status );
}

//
// PRIVATE METHODS
// ---------------------------------------------------------------------------
//bool I2CIO::isAvailable (uint8_t i2cAddr)
//{
//int error;
//
//Wire.beginTransmission( i2cAddr );
//error = Wire.endTransmission();
//if (error==0)
//{
//return true;
//}
//else //Some error occured
//{
//return false;
//}
//}
