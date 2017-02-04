/* Copyright 2016, Fernandez Martin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef REGISTRADOR_GPS_DRIVER_H_
#define REGISTRADOR_GPS_DRIVER_H_

/*==========================[inclusions]=================================*/

#include "../../Drivers/inc/UART_driver.h"
#include "board.h"
#include "chip.h"

/*======================[functions declaration]=========================*/

/**
 * @brief	GPS module initialization
 * @param	n		: UART number, should be 0, 2 or 3
 * @return	Nothing
 * @note	Initializes UART peripheral and sends out NMEA configuration
 * 			messages for the module
 */
void GPS_Init(uarts_e n);

/**
 * @brief	GPS module deinitialization
 * @return	Nothing
 * @note	Puts the GPS on standby and deinitializes UART peripheral
 */
void GPS_DeInit(void);

/**
 * @brief	GPS general application
 * @return	Nothing
 * @note	Required function for the GPS driver to work. Reads data from
 * 			the UART ring buffer and analyses it, saving the data from different
 * 			NMEA sentences into structs
 */
void GPS_App(void);

/**
 * @brief	Sends out NMEA configuration message to enable hot restart
 * @return	Nothing
 * @note  	Hot Restart: Use all available data in the NV Store
 */
void GPS_HotRestart(void);
/**
 * @brief	Sends out NMEA configuration message to reset the GPS to factory values
 * @return	Nothing
 * @note 	Full Cold Restart: Don't use Time, Position, Almanacs and Ephemeris data at re-start.
 * 			Additionally clear system/user configurations at re-start. That is, reset the receiver to the
 * 			factory status.
 */
void GPS_FactoryReset(void);
/**
 * @brief	Sends out NMEA configuration message to put the GPS in standby
 * @return	Nothing
 */
void GPS_Standby(void);
/**
 * @brief	Wakes GPS from standby
 * @return	Nothing
 */
void GPS_Wakeup(void);

/**
 * @brief	Checks if there is new data available from the GPS
 * @return	True	: There is new data available
 * @return	False	: There is no new data available
 */
Bool GPS_DataReady(void);

/**
 * @brief	Clears gps data status register
 * @return	Nothing
 */
void GPS_ClearData(void);

/**
 * @brief	Returns measured speed in metres per second
 * @return	0	: data successfully copied to variable used as argument for the function
 * @return	-1	: there is no new speed data available to be copied
 */
int32_t GPS_Speed_MKS(float * data);

/**
 * @brief	Returns measured altitude in metres
 * @return	0	: data successfully copied to variable used as argument for the function
 * @return	-1	: there is no new altitude data available to be copied
 */
int32_t GPS_Altitude_metres(float * data);

/**
 * @brief	Returns measured latitude
 * @return	0	: data successfully copied to variables used as argument for the function
 * @return	-1	: there is no new latitude data available to be copied
 */
int32_t GPS_Latitude_deg(float * deg, uint8_t * dir);

/**
 * @brief	Returns measured longitude
 * @return	0	: data successfully copied to variables used as argument for the function
 * @return	-1	: there is no new longitude data available to be copied
 */
int32_t GPS_Longitude_deg(float * deg, uint8_t * dir);

/**
 * @brief	Returns measured time in seconds
 * @return	0	: data successfully copied to variable used as argument for the function
 * @return	-1	: there is no new altitude data available to be copied
 * @note	Time data is stored in seconds. It can be converted from seconds to hh:mm:ss
 * 			format using simple operations
 */
int32_t GPS_Time_sec(uint32_t * data);


/*=========================[global data declaration]==============================*/

/* --------------------- Global data types of NMEA sentences ------------------------------ */

typedef struct _timeData
{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
}timeData_t;

typedef struct _orientationData
{
	uint32_t deg;
	float min;
	uint8_t direction;
}orientationData_t;

typedef struct _dateData
{
	uint8_t day;
	uint8_t month;
	uint8_t year;
}dateData_t;

typedef struct _magneticData
{
	float value;
	uint8_t direction;
}magneticData_t;

typedef enum _gpsFix
{
	Invalid = 0,				// Invalid Gps fix
	GpsFix = 1,					// Gps fix
	DGpsFix = 2,				// Differential Gps fix
	PpsFix = 3,					// PPS fix
	RealTimeKinematic = 4,		// Real Time Kinematic (RTK) fix
	FloatRealTimeKinematic = 5,	// Float Real Time Kinematic (Float RTK) fix
	Estimated = 6,				// Estimated fix
	ManualInputMode = 7,		// Manual fix
	SimulationMode = 8,			// Simulated fix
}gpsFix_e;

#endif /* REGISTRADOR_GPS_DRIVER_H_ */
