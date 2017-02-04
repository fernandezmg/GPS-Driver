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

/*==================[inclusions]=============================================*/

#include "../../Drivers/inc/GPS_driver.h"

#include <string.h>
#include <stdlib.h>
#include "../../Drivers/inc/UART_driver.h"

/*==================[macros and definitions]=================================*/

/* TX Messages */
#define  MT3337_SET_STDBY_CMD    "$PMTK161,0*28\r\n"    // Set standby
#define  MT3337_HOT_RST_CMD      "$PMTK101*32\r\n"      // Hot restart
#define  MT3337_WARM_RST_CMD     "$PMTK102*31\r\n"      // Warn Restart
#define  MT3337_COLD_RST_CMD     "$PMTK103*30\r\n"      // Cold restart
#define  MT3337_FCOLD_RST_CMD    "$PMTK104*37\r\n"      // Full cold restart
#define  MT3337_CLR_FLASH_CMD    "$PMTK120*31\r\n"      // Clear flash
#define  MT3337_CLEAR_ORBIT_CMD  "$PMTK127*36\r\n"      // Clear predicted orbit data
#define  MT3337_SET_FIXINT_CMD   "$PMTK220,1000*1F\r\n" // Set position fix interval to 1000
#define  MT3337_ENABLE_PPS_SYNC	 "$PMTK255,1*2D\r\n" 	// Enable PPS Sync

#define  MT3337_CONF_CMD 		 "PMTK"
#define  MT3337_GPRMC_WORD		 "GPRMC"
#define  MT3337_GPGGA_WORD		 "GPGGA"

/* Sets the NMEA stream to only send RMC and GGA string
 * See: PMTK_Protocol -> "Packet Type: 314 PMTK_API_SET_NMEA_OUTPUT" */
#define  MT3337_SET_NMEA_OUTPUT	 "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"

#define STATUS_ACTIVE	'A'
#define MIN_DEG_CONV 0.0166667
#define KNOTS_METRES_CONV 0.5144444
#define MIN_SEC_CONV 60
#define HOUR_SEC_CONV 3600


#define BUFFER_SIZE 1024
#define FIELD_SIZE 16 // some of the resulting strings were 9 chars long
#define FIELD_NUMBER 32 // max number of fields currently used = 16


/* ----------------------------- GPS Data Status Register  ------------------------------ */

#define DATE_DATA					(0x1 << 0)
#define TIME_DATA					(0x1 << 1)
#define LONGITUDE_DATA				(0x1 << 2)
#define LATITUDE_DATA				(0x1 << 3)
#define SPEED_DATA					(0x1 << 4)
#define ALTITUDE_DATA				(0x1 << 5)

/*=========================[internal data declaration]==============================*/

/* ----------------------------- UART driver interface ------------------------------ */

static uarts_e uart_gps;

/* ----------------------------- RMC Data Struct ------------------------------ */

//RMC - NMEA has its own version of essential gps pvt (position, velocity, time) data.
//It is called RMC, The Recommended Minimum, which will look similar to:
//
//$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
//
//Where:
//     RMC          Recommended Minimum sentence C
//     123519       Fix taken at 12:35:19 UTC
//     A            Status A=active or V=Void.
//     4807.038,N   Latitude 48 deg 07.038' N
//     01131.000,E  Longitude 11 deg 31.000' E
//     022.4        Speed over the ground in knots
//     084.4        Track angle in degrees True
//     230394       Date - 23rd of March 1994
//     003.1,W      Magnetic Variation
//     *6A          The checksum data, always begins with *

typedef struct _gpsRMCData
{
	timeData_t time;
	uint8_t status;
	orientationData_t latitude;
	orientationData_t longitude;
	float speed;
	float angle;
	dateData_t date;
	magneticData_t magnetic;
	uint32_t checksum;
}gpsRMCData_t;

/* ----------------------------- GGA Data Struct ------------------------------ */

//GGA - essential fix data which provide 3D location and accuracy data.
//
// $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
//
//Where:
//     GGA          Global Positioning System Fix Data
//     123519       Fix taken at 12:35:19 UTC
//     4807.038,N   Latitude 48 deg 07.038' N
//     01131.000,E  Longitude 11 deg 31.000' E
//     1            Fix quality: 0 = invalid
//                               1 = GPS fix (SPS)
//                               2 = DGPS fix
//                               3 = PPS fix
//			      				 4 = Real Time Kinematic
//			    				 5 = Float RTK
//            				     6 = estimated (dead reckoning) (2.3 feature)
//			  				     7 = Manual input mode
//			    				 8 = Simulation mode
//     08           Number of satellites being tracked
//     0.9          Horizontal dilution of position
//     545.4,M      Altitude, Meters, above mean sea level
//     46.9,M       Height of geoid (mean sea level) above WGS84
//                      ellipsoid
//     (empty field) time in seconds since last DGPS update
//     (empty field) DGPS station ID number
//     *47          the checksum data, always begins with *
//
//	 If the height of geoid is missing then the altitude should be suspect. Some non-standard
//	 implementations report altitude with respect to the ellipsoid rather than geoid altitude.
//	 Some units do not report negative altitudes at alUART driver interface. This is the only
//	 sentence that reports altitude.

typedef struct _gpsGGAData
{
	timeData_t time;
	orientationData_t latitude;
	orientationData_t longitude;
	gpsFix_e fixstatus;
	uint8_t satelitecount;
	float dilutionofposition;
	float altitude;
	float heigthofgeoid;
	uint32_t checksum;

}gpsGGAData_t;

/* ----------------------------- Data declaration ------------------------------ */

static struct _gpsRMCData outputrmc;
static struct _gpsGGAData outputgga;

/* ---------------- GPS Data Status Register -------------------
 * brief: shows what new valid data is available
 *
 * Bit		Description
 *
 * 0		Date
 * 1		Time
 * 2		Longitude
 * 3		Latitude
 * 4		Speed
 * 5		Altitude
 * 6:7		Unused			 */
static uint8_t gpsdataSTR = 0;

/*==================[internal functions declaration]=========================*/

/**
 * @brief	Sends out NMEA configuration message to enable PPS sync
 * @return	Nothing
 * @note 	Enable fix NMEA output time behind PPS function
 */
static void GPS_EnablePPSSync(void);

/**
 * @brief	Sends out NMEA configuration message to set the NMEA stream to only
 * 			send the RMC and GGA strings
 * @return	Nothing
 */
static void GPS_SetNMEAOutput(void);

/**
 * @brief	Gets data from rx ring buffer and sends NMEA sentences one at a time to the parser
 * @param	* data	: Pointer to received data buffer
 * @param	nbytes	: number of received bytes
 * @return	0	: Data was sent successfully
 * @return	-1	: Data couldn't be sent
 */
static int32_t GPS_Parser(uint8_t * data, uint32_t nbytes);

/**
 * @brief	Parses an NMEA sentence and calls allocating functions
 * @param	* data	: Pointer to NMEA sentence to be parsed and stored
 * @param	length	: length of NMEA sentence to be be parsed and stored
 * @return	0	:	Parsing was completed successfully
 * @return	-1	:	There was an error in the parsing and/or storing process
 */
static int32_t GPS_StoreData(uint8_t * data, uint32_t length);

/**
 * @brief	Allocates RMC sentence into its struct
 * @param	data[][FIELD_SIZE]	: Matrix holding the parsed data ready to be stored
 * @return	Nothing
 */
static void GPS_AllocateRMCData(uint8_t data[][FIELD_SIZE]);

/**
 * @brief	Allocates GGA sentence into its struct
 * @param	data[][FIELD_SIZE]	: Matrix holding the parsed data ready to be stored
 * @return	Nothing
 */
static void GPS_AllocateGGAData(uint8_t data[][FIELD_SIZE]);

/**
 * @brief	Converts "length" bytes of a string into an uint32_t number
 * @param	* string	: Pointer to string to be converted
 * @param	length		: string length in bytes
 * @return	retval	:	Converted uint32_t value
 */
static uint32_t GPS_strtoint(uint8_t * string, uint32_t length);

/*==================[internal functions definition]==========================*/

static int32_t GPS_Parser(uint8_t * data, uint32_t nbytes)
{
	int32_t success = 0, error = -1;

	uint32_t n, i, bytesleft;
	uint8_t * ptr;

	for(n = 0 ; n < nbytes ; n++)
	{
		ptr = data + n;

		if(*ptr == '$')  // beginning of NMEA sentence detected
		{
			i = 0;

			ptr++; // I don't want to copy the '$'
			bytesleft = nbytes - n - 1;

			/*moved i index to the end of the word*/
			while((*(ptr + i) != '\n') & (i != bytesleft)) i++;

			if(i == bytesleft)
			{
				return error;
			}

			GPS_StoreData(ptr, i);

			n += i; // we add i to n to avoid unnecessary processing cycles
		}
	}

	return success;
}


static int32_t GPS_StoreData(uint8_t * data, uint32_t length)
{
	int32_t success = 0, error = -1;

	uint32_t i, nNUL = 0;
	uint8_t nmeasentence[length], * ptr;
	char * destination, * origin, * idfield;
	/* matrix used to store the different fields as string data of a NMEA sentence */
	uint8_t stringgpsvec[FIELD_NUMBER][FIELD_SIZE] = {0};

	memcpy(nmeasentence, data, length);

	/* Copied data looks like this (we use RMC sentence data as an example; should work for any sentence) ->
	 * "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r" */

	/* memchr -> locates character in block of memory. Returns pointer to the first occurrence of searched character.
	 * If the character couldn't be found returns pointer to NULL.
	 */
	ptr = memchr(nmeasentence, ',', length);

	if(ptr == NULL)
	{
		return error;
	}

	do
	{
		*ptr = '\0';
		nNUL++;
		ptr = memchr(nmeasentence, ',', length);
	} while(ptr != NULL);

	/* Now data looks like this ->
	 * "GPRMC\0123519\0A\04807.038\0N\001131.000\0E\0022.4\0084.4\0230394\0003.1\0W*6A\r" */

	ptr = memchr(nmeasentence, '*', length);

	if(ptr == NULL)
	{
		/* if '*' could not be found something is wrong with the sentence */
		return error;
	}

	*ptr = '\0';
	nNUL++;

	ptr = memchr(nmeasentence, '\r', length);

	if(ptr == NULL)
	{
		/* if '\r' could not be found something is wrong with the sentence */
		return error;
	}

	*ptr = '\0';
	nNUL++;

	/* Now the data looks like this ->
	 * "GPRMC\0123519\0A\04807.038\0N\001131.000\0E\0022.4\0084.4\0230394\0003.1\0W\06A\0" */

	origin = (char *) nmeasentence; // This goes outside the loop. Otherwise it'd always go back to the beginning

	for(i = 0 ; i < nNUL ; i++)
	{
		destination = (char *) stringgpsvec[i];

		/* if \0 is found before FIELD_SIZE is reached the rest is padded with zeroes */
		strncpy(destination, origin, FIELD_SIZE);

		ptr = memchr(destination, '\0', FIELD_SIZE);

		/* if '\0' couldn't be found we break the loop to avoid strlen from overflowing */
		/*If the character couldn't be found ptr == NULL.*/
		if(ptr == NULL)
		{
			return error;
		}

		length = strlen(destination) + 1; /* strlen("Hello") = 5 -> 'Hello\0' = 6 */
		origin += length; /*moved to the next word*/
	}

	/* Now the data is stored as strings inside a vector. Eg: Field[0] = "GPRMC" and so on */

	/* we check to see which NMEA sentence was received */
	idfield = (char *) stringgpsvec[0];

	if(!strcmp(idfield, MT3337_GPRMC_WORD))
	{
		GPS_AllocateRMCData(stringgpsvec);

		/* if the received data was valid we write the gps status register to reflect
		 * which new data has been copied  */
		if(outputrmc.status == STATUS_ACTIVE)
		{
			gpsdataSTR |= (DATE_DATA | TIME_DATA | LONGITUDE_DATA | LATITUDE_DATA | SPEED_DATA);
		}

		return success;

	}
	else if(!strcmp(idfield, MT3337_GPGGA_WORD))
	{
		GPS_AllocateGGAData(stringgpsvec);

		/* we can also extract longitude and latitude data from the gga sentence, but we are not using it
		 * for this application. In the future the ability to extract other fields from this and other
		 * sentences can be added. Simply expand the gpsdataSTR to allow more data entries, and OR the
		 * other data that's being collected bellow */

		/* if the received data was valid we write the gps status register to reflect
		 * which new data has been copied  */
		if(outputgga.fixstatus == GpsFix || outputgga.fixstatus == DGpsFix || outputgga.fixstatus == PpsFix)
		{
			gpsdataSTR |= (ALTITUDE_DATA);
		}

		return success;
	}
	else
	{
		return error;
	}

	/* data is now stored in numerical form in its respective structure */
}


static void GPS_AllocateRMCData(uint8_t data[][FIELD_SIZE])
{
	uint8_t * ptr;

//$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
//$GPRMC,001430,A,3907.3885,N,12102.4767,W,414.0,175.3,220403,015.4,E*71

//	data[0] = "GPRMC\0"
//	data[1] = "123519\0"
//	data[2] = "A\0"
//	data[3] = "4807.038\0"
//	data[4] = "N\0"
//	data[5] = "01131.000\0"
//	data[6] = "E\0"
//	data[7] = "022.4\0"
//	data[8] = "084.4\0"
//	data[9] = "230394\0"
//	data[10] = "003.1\0"
//	data[11] = "W\0"
//	data[12] = "6A\0"

	ptr = data[1];
	outputrmc.time.hours = GPS_strtoint(ptr, 2);
	ptr += 2;
	outputrmc.time.minutes = GPS_strtoint(ptr,2);
	ptr += 2;
	outputrmc.time.seconds = GPS_strtoint(ptr,2);
	ptr = data[2];
	outputrmc.status = *ptr;
	ptr = data[3];
	outputrmc.latitude.deg = GPS_strtoint(ptr,2);
	ptr += 2;
	outputrmc.latitude.min = strtof((char *)ptr, NULL);
	ptr = data[4];
	outputrmc.latitude.direction = *ptr;
	ptr = data[5];
	outputrmc.longitude.deg = GPS_strtoint(ptr,3);
	ptr += 3;
	outputrmc.longitude.min = strtof((char *)ptr, NULL);
	ptr = data[6];
	outputrmc.longitude.direction = *ptr;
	ptr = data[7];
	outputrmc.speed = strtof((char *)ptr, NULL);
	ptr = data[8];
	outputrmc.angle = strtof((char *)ptr, NULL);
	ptr = data[9];
	outputrmc.date.day = GPS_strtoint(ptr, 2);
	ptr += 2;
	outputrmc.date.month = GPS_strtoint(ptr, 2);
	ptr += 2;
	outputrmc.date.year = GPS_strtoint(ptr, 2);
	ptr = data[10];
	outputrmc.magnetic.value = strtof((char *)ptr, NULL);
	ptr = data[11];
	outputrmc.magnetic.direction = *ptr;
	ptr = data[12];
	outputrmc.checksum = strtoul((char *)ptr, NULL, 16); /* Checksum is a hexadecimal number */
}


static void GPS_AllocateGGAData(uint8_t data[][FIELD_SIZE])
{
	uint8_t * ptr;

// $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

// $GPGGA,001431.003,3907.3885,N,12102.4767,W,1,05,02.1,00545.5,M,-26.0,M,,*5P

//	data[0] = "GPGGA\0"
//	data[1] = "123519\0"
//	data[2] = "4807.038\0"
//	data[3] = "N\0"
//	data[4] = "01131.000\0"
//	data[5] = "E\0"
//	data[6] = "1\0"
//	data[7] = "08\0"
//	data[8] = "0.9\0"
//	data[9] = "545.4\0"
//	data[10] = "M\0"
//	data[11] = "46.9\0"
//	data[12] = "M\0"
//	data[13] = "\0"
//	data[14] = "\0"
//	data[15] = "47\0"

	ptr = data[1];
	outputgga.time.hours = GPS_strtoint(ptr, 2);
	ptr += 2;
	outputgga.time.minutes = GPS_strtoint(ptr, 2);
	ptr += 2;
	outputgga.time.seconds = GPS_strtoint(ptr,2);
	ptr = data[2];
	outputgga.latitude.deg = GPS_strtoint(ptr,2);
	ptr += 2;
	outputgga.latitude.min = strtof((char *)ptr, NULL);
	ptr = data[3];
	outputgga.latitude.direction = *ptr;
	ptr = data[4];
	outputgga.longitude.deg = GPS_strtoint(ptr,3);
	ptr += 3;
	outputgga.longitude.min = strtof((char *)ptr, NULL);
	ptr = data[5];
	outputgga.longitude.direction = *ptr;
	ptr = data[6];
	outputgga.fixstatus = strtoul((char *)ptr, NULL, 10);
	ptr = data[7];
	outputgga.satelitecount = strtoul((char *)ptr, NULL, 10);
	ptr = data[8];
	outputgga.dilutionofposition = strtof((char *)ptr, NULL);
	ptr = data[9];
	outputgga.altitude = strtof((char *)ptr, NULL);
	ptr = data[11];
	outputgga.heigthofgeoid = strtof((char *)ptr, NULL);
	ptr = data[15];
	outputgga.checksum = strtoul((char *)ptr, NULL, 16);
}


static uint32_t GPS_strtoint(uint8_t * string, uint32_t length)
{
	uint32_t retval;
	char value[FIELD_SIZE];
	memcpy(value, string, length);
	value[length] = '\0'; // marks the end of the data to be converted (strtoul works with strings)
	retval = strtoul(value, NULL, 10);
	return retval;
}

static void GPS_EnablePPSSync(void)
{
	uint8_t msg[] = MT3337_ENABLE_PPS_SYNC;
	UART_WriteNBytes(uart_gps, msg, sizeof(msg));
}


static void GPS_SetNMEAOutput(void)
{
	uint8_t msg[] = MT3337_SET_NMEA_OUTPUT;
	UART_WriteNBytes(uart_gps, msg, sizeof(msg));
}

/*==================[external functions definition]==========================*/

void GPS_Init(uarts_e n)
{
	uart_gps = n;
	UART_Init(uart_gps);
	// FIXME: Se requiere un tiempo para que funcionen los mensajes de configuracion (???)
	for( int i = 0 ; i < 0xffff ; i++ ); // delay 5 dedos oscilantes
	GPS_EnablePPSSync();
	GPS_SetNMEAOutput();
}

void GPS_DeInit(void)
{
	GPS_Standby();
	UART_DeInit(uart_gps);
}

void GPS_App(void)
{
	uint32_t nbytes;
	uint8_t streambuf[BUFFER_SIZE];

	memset(streambuf, 0, BUFFER_SIZE);

	if(UART_CheckRxEmpty(uart_gps)) // there is no new data being received (Uart Rx idle)
	{
		nbytes = UART_DataCount(uart_gps);

		if(nbytes != 0)
		{
			if (nbytes > BUFFER_SIZE)
				nbytes = BUFFER_SIZE;

			UART_GetNBytes(uart_gps, streambuf, nbytes);
			UART_WriteNBytes(UART3, streambuf, nbytes);
			GPS_Parser(streambuf, nbytes);
		}
	}
}


Bool GPS_DataReady(void)
{
	if(gpsdataSTR != 0) /* one or more bits are 1 */
	{
		return TRUE;
	}

	return FALSE;
}

void GPS_ClearData(void)
{
	gpsdataSTR = 0;
}

int32_t GPS_Speed_MKS(float * data)
{
	int32_t success = 0, error = -1;

	/* verifies if there is new speed data */
	if(!(gpsdataSTR & SPEED_DATA))
	{
		return error;
	}

	/* 1 knot is equal to 0.514444444444 metre/second */
	*data = outputrmc.speed * KNOTS_METRES_CONV;

	gpsdataSTR &= ~(SPEED_DATA); // NOT operator (~)

	return success;
}

int32_t GPS_Altitude_metres(float * data)
{
	int32_t success = 0, error = -1;

	/* verifies if there is new altitude data */
	if(!(gpsdataSTR & ALTITUDE_DATA))
	{
		return error;
	}

	*data = outputgga.altitude;

	gpsdataSTR &= ~(ALTITUDE_DATA); // NOT operator (~)

	return success;
}

int32_t GPS_Latitude_deg(float * deg, uint8_t * dir)
{
	int32_t success = 0, error = -1;

	/* verifies if there is new latitude data */
	if(!(gpsdataSTR & LATITUDE_DATA))
	{
		return error;
	}

	*deg = outputrmc.latitude.deg + outputrmc.latitude.min * MIN_DEG_CONV;
	*dir = outputrmc.latitude.direction;

	gpsdataSTR &= ~(LATITUDE_DATA); // NOT operator (~)

	return success;

}

int32_t GPS_Longitude_deg(float * deg, uint8_t * dir)
{
	int32_t success = 0, error = -1;

	/* verifies if there is new longitude data */
	if(!(gpsdataSTR & LONGITUDE_DATA))
	{
		return error;
	}

	*deg = outputrmc.longitude.deg + outputrmc.longitude.min * MIN_DEG_CONV;
	*dir = outputrmc.longitude.direction;

	gpsdataSTR &= ~(LONGITUDE_DATA); // NOT operator (~)

	return success;

}

int32_t GPS_Time_sec(uint32_t * data)
{
	int32_t success = 0, error = -1;

	/* verifies if there is new time data */
	if(!(gpsdataSTR & TIME_DATA))
	{
		return error;
	}

	*data = outputrmc.time.seconds + outputrmc.time.minutes * MIN_SEC_CONV + outputrmc.time.hours * HOUR_SEC_CONV;

	gpsdataSTR &= ~(TIME_DATA); // NOT operator (~)

	return success;
}

void GPS_HotRestart(void)
{
	uint8_t msg[] = MT3337_HOT_RST_CMD;
	UART_WriteNBytes(uart_gps, msg, sizeof(msg));
}

void GPS_FactoryReset(void)
{
	uint8_t msg[] = MT3337_FCOLD_RST_CMD;
	UART_WriteNBytes(uart_gps, msg, sizeof(msg));
}

void GPS_Standby(void)
{
	uint8_t msg[] = MT3337_SET_STDBY_CMD;
	UART_WriteNBytes(uart_gps, msg, sizeof(msg));
}

void GPS_Wakeup(void)
{
	/* Any byte wakes the module up from standby mode [datasheet M10478-A2-1 page 10] */
	uint8_t msg[] = "w\r\n";
	UART_WriteNBytes(uart_gps, msg, sizeof(msg));
}
