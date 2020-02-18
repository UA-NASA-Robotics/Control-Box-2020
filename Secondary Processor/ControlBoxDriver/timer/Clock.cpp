/*
 * Clock.cpp
 *
 * Created: 1/3/2019 1:04:11 PM
 *  Author: Seth Carpenter
 */

#include "Clock.hpp"

#include "../timer/SimpleTimer1.hpp"
#include "../uart/uart_2.hpp"
#include "../uart/uart_3.hpp"

static SimpleTimer1 ClockTimer;
//Library constructor
Clocks::Clocks()
{
	ClockTimer.set_duration(1);
	ClockTimer.start();
	length=50;
	reset();
}
Clocks::Clocks(long l)
{
	ClockTimer.set_duration(1);
	ClockTimer.start();
	length=l;
	reset();
}
//External method
void Clocks::reset()
{
	startTime=ClockTimer.millis();
	currentTime=startTime;
}
void Clocks::resetSpecial()
{
	startTime=startTime + length;
	currentTime=ClockTimer.millis();
}
void Clocks::setInterval(long l)
{
	length=l;
}
void Clocks::update()
{
	currentTime=ClockTimer.millis();
}
bool Clocks::isDone()
{
	update();
	if (abs(currentTime-startTime)>length)
	{
		reset();
		return true;
	}
	else {
		return false;
	}
}
bool Clocks::isDoneSpecial()
{
	update();
	if (abs(currentTime-startTime)>length)
	{
		resetSpecial();
		return true;
	}
	else {
		return false;
	}
}
bool Clocks::isDone_NoReset()
{
	update();
	if (abs(currentTime-startTime)>length)
	{
		return true;
	}
	else {
		return false;
	}
}
unsigned long Clocks::GetElapsedTime()
{
	update();

	return abs(currentTime-startTime) / length;

}
