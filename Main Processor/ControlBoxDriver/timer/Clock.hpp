/*
 * Clocks.h
 *
 * Created: 1/3/2019 2:15:09 PM
 *  Author: John
 */


#ifndef CLOCKS_H_
#define CLOCKS_H_

/*
 * Clock.h
 *
 * Created: 1/3/2019 1:04:22 PM
 *  Author: John
 */


#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdint-gcc.h>
#include <stdlib.h>

class Clocks
{
public:
	Clocks();
	Clocks(long l);
	void reset();
	void resetSpecial();
	void setInterval(long l);
	void update();
	bool isDone();
	bool isDoneSpecial();
	bool isDone_NoReset();
	unsigned long GetElapsedTime();
	long currentTime;
	long startTime;
	long length;
private:


};


#endif /* CLOCK_H_ */



#endif /* CLOCKS_H_ */