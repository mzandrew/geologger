// written 2022-05-07 by mza
// based on Example17_NTRIPClient_With_GGA_Callback by sparkfun, but gutted
// yet more from adafruit_04_location
// more from https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
// also from https://github.com/adafruit/SdFat/blob/master/examples/OpenNext/OpenNext.ino
// rtc stuff from https://github.com/adafruit/RTClib/blob/master/examples/pcf8523/pcf8523.ino
// last updated 2023-04-30 by mza

#include <stdint.h>
#include <SPI.h>
#include <RH_RF95.h>

#include <SdFat.h>
SdFat sd;
SdFile log_file;
const char log_file_name[] = "geologger.log";
#define MICROSD_CS (10)
int total_number_of_datapoints_in_file = 0;

#include "RTClib.h"
RTC_PCF8523 rtc;

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
	#include "base64.h" //Built-in ESP32 library
#else
	#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

uint8_t verbosity = 4; // debug2=5; debug=4; info=3; warning=2; error=1
#define RSSI_THRESHOLD (-150)
#define JUNK_RSSI (-151)
#define LORA_PING_PONG_TIMEOUT_IN_MILLISECONDS (1000)
#define NAVIGATION_FREQUENCY_HZ (5)
#define SCREEN_UPDATE_TIMEOUT_IN_MILLISECONDS (1000/NAVIGATION_FREQUENCY_HZ)
#define UPLOAD_TIMEOUT_IN_MILLISECONDS (6000)
#define MAX_UPLOADS (99)
#define MAX_UPLOAD_RATE_PER_MINUTE (10)
#define MINIMUM_HORIZONTAL_ACCURACY_MM (100)
#define ACQUIRE_LORA_RSSI_DATA
//#define POST_LORA_RSSI_DATA_OVER_LORA
//#define DEBUG_LORA_RSSI
#define LORA_TX_POWER (20) // [5,20]

#ifdef ACQUIRE_LORA_RSSI_DATA
	#define USE_LORA
	int lora_rssi_ping = JUNK_RSSI;
	int lora_rssi_pong = JUNK_RSSI;
#endif

unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned long pingPongTime = 0;
unsigned long screenUpdateTime = 0;
unsigned long screenUpdateMiniTime = 0;
unsigned long uploadTime = 0;

#define NODEID (4)
#define MAX_STRING_LENGTH (255) // uint8_t RH_RF95::maxMessageLength()
const char PREFIX[]="SCOOPY";
const char SUFFIX[]="BOOPS";
uint8_t recvpacket[MAX_STRING_LENGTH];
char received_message[MAX_STRING_LENGTH];
char sendpacket1[MAX_STRING_LENGTH-strlen(PREFIX)-strlen(SUFFIX)] = "nothing to see here";
char sendpacket2[MAX_STRING_LENGTH] = "this is a dummy message";
char sendpacket3[MAX_STRING_LENGTH];

#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT
	#define ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT_OR_REVTFT
#endif
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_REVTFT
	#define ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT_OR_REVTFT
#endif
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT_OR_REVTFT
	//#define RFM95_INT (10) // B = IRQ
	//#define RFM95_CS   (5) // E = CS
	//#define RFM95_RST  (6) // D = reset
	#define RFM95_INT (12) // GPIO12 = D12 = IRQ
	#define RFM95_CS  (13) // GPIO13 = D13 = CS
	#define RFM95_RST  (8) // F = GPIO8 = A5 = reset
#endif
#define LORA_FREQ (905.0)
RH_RF95 lora(RFM95_CS, RFM95_INT);
bool lora_is_available = false;
bool setup_lora(void);

//#define BUTTON1 (18) // GPIO18 = A0
#define BUTTON1 (17) // GPIO17 = A1
#define BUTTON2 (16) // GPIO16 = A2
bool previous_button1 = false;
bool previous_button2 = false;
unsigned long previous_button1_change_time = 0;
unsigned long previous_button2_change_time = 0;
#define BUTTON_DEBOUNCE_TIME (50) // in milliseconds

//#define STMPE_CS 6
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT_OR_REVTFT
	#include <Adafruit_ST7789.h>
	Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
	#define TFT_WIDTH  (240)
	#define TFT_HEIGHT (135)
	#define CURSOR_X (0)
	#define CURSOR_Y (4)
#endif

#define LENGTH_OF_LINE (21)
#define LENGTH_OF_SSID (LENGTH_OF_LINE-4)
#define NUMBER_OF_LINES (8)
char line[NUMBER_OF_LINES][LENGTH_OF_LINE+5];
char paragraph[NUMBER_OF_LINES*LENGTH_OF_LINE+1];
char datestamp[30] = "";

uint8_t fixType = 0;
String fixTypeString[] = { "none", "dead_reck", "2d", "3d", "gnss+dead_reck", "time_only", "unknown" };
#define MAX_VALID_FIXTYPE (6)
uint8_t carrSoln = 0;
String carrSolnString[] = { "none", "floating", "fixed", "unknown" };
#define MAX_VALID_CARRSOLN (3)
#define JUNK_LAT (44.444444)
#define JUNK_LON (-77.777777)
#define JUNK_ELE (1.111111)
double lat = JUNK_LAT;
double lon = JUNK_LON;
double ele = JUNK_ELE;
double lora_lat = JUNK_LAT;
double lora_lon = JUNK_LON;
double lora_ele = JUNK_ELE;
#define JUNK_ACC_MM (10000)
uint32_t lora_hAcc_mm = JUNK_ACC_MM;
uint32_t hAcc_mm = JUNK_ACC_MM;
uint32_t vAcc_mm = JUNK_ACC_MM;
uint8_t diffSoln = 0;
uint8_t numSV = 0;
uint16_t pDOP = 9999;
int32_t height_mm = 0;
uint16_t year   = 70; // Year (UTC)
uint8_t  month  = 70; // Month, range 1..12 (UTC)
uint8_t  day    = 70; // Day of month, range 1..31 (UTC)
uint8_t  hour   = 70; // Hour of day, range 0..23 (UTC)
uint8_t  minute = 70; // Minute of hour, range 0..59 (UTC)
uint8_t  second = 70; // Seconds of minute, range 0..60 (UTC)
uint8_t current_minute = 70; // for upload counter
uint32_t number_of_uploads_for_the_current_minute = 0;
uint32_t total_number_of_uploads = 0;
uint32_t total_number_of_pings_sent = 0;
uint32_t total_number_of_pongs_received = 0;

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |

void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
	year = ubxDataStruct->year;
	month = ubxDataStruct->month;
	day = ubxDataStruct->day;
	hour = ubxDataStruct->hour;
	minute = ubxDataStruct->min;
	second = ubxDataStruct->sec;
	if (current_minute!=minute) {
		number_of_uploads_for_the_current_minute = 0;
		current_minute = minute;
	}
	double latitude = ubxDataStruct->lat; // Print the latitude
	double longitude = ubxDataStruct->lon; // Print the longitude
	double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
	fixType = ubxDataStruct->fixType; // Print the fix type
	if (MAX_VALID_FIXTYPE<fixType) {
		fixType = MAX_VALID_FIXTYPE;
	}
	carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution
	if (MAX_VALID_CARRSOLN<carrSoln) {
		fixType = MAX_VALID_CARRSOLN;
	}
	hAcc_mm = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
	diffSoln = ubxDataStruct->flags.bits.diffSoln; // 1 = differential corrections were applied
	numSV = ubxDataStruct->numSV;  // Number of satellites used in Nav Solution
	pDOP = ubxDataStruct->pDOP;  // Position DOP * 0.01
	height_mm = ubxDataStruct->height; // Height above ellipsoid: mm
	vAcc_mm = ubxDataStruct->vAcc;  // Vertical accuracy estimate: mm
	sprintf(paragraph, "Lat: %.8f Long: %.8f Height: %.3f numSv: %d Fix: %d %s carrSoln: %d %s diffSoln: %d hAcc_mm: %d vAcc_mm: %d", latitude / 10000000.0, longitude / 10000000.0, altitude / 1000.0, numSV, fixType, fixTypeString[fixType].c_str(), carrSoln, carrSolnString[carrSoln].c_str(), diffSoln, hAcc_mm, vAcc_mm);
	Serial.println(paragraph);
	lat = latitude / 10000000.0;
	lon = longitude / 10000000.0;
	ele = altitude / 1000.0;
}

void debug2(const char *message) {
	if (5<=verbosity) {
		Serial.print("DEBUG2: ");
		Serial.println(message);
	}
}

void debug(const char *message) {
	if (4<=verbosity) {
		Serial.print("DEBUG: ");
		Serial.println(message);
	}
}

void info(const char *message) {
	if (3<=verbosity) {
		Serial.println(message);
	}
}

void warning(const char *message) {
	if (2<=verbosity) {
		Serial.print("WARNING: ");
		Serial.println(message);
	}
}

void error(const char *message) {
	if (1<=verbosity) {
		Serial.print("ERROR: ");
		Serial.println(message);
	}
}

bool lf(String location) {
	SdFile file;
	SdFile dir;
	bool return_value = true;
	if (dir.open(location.c_str())){
		while (file.openNext(&dir, O_RDONLY)) {
			if (!file.isDir()) {
				file.printModifyDateTime(&Serial);
				Serial.print(" ");
				file.printFileSize(&Serial);
				Serial.print(" ");
				file.printName(&Serial);
				Serial.println();
			}
			file.close();
		}
	} else {
		error("dir.open failed");
		return_value = false;
	}
	return return_value;
}

void setup() {
	startTime = millis();
	Serial.begin(115200);
	if (1) {
		delay(4000);
		Serial.println("----------------------------------------------------------");
		sprintf(paragraph, "geologger");
		Serial.println(paragraph); //tft.println(paragraph);
		sprintf(paragraph, "startTime: %ld", startTime);
		Serial.println(paragraph); //tft.println(paragraph);
	}
	#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT_OR_REVTFT
		pinMode(RFM95_RST, OUTPUT);
		digitalWrite(RFM95_RST, LOW);
		delay(100);
		pinMode(RFM95_RST, OUTPUT);
		digitalWrite(RFM95_RST, HIGH);
		delay(100);
		pinMode(TFT_I2C_POWER, OUTPUT);
		digitalWrite(TFT_I2C_POWER, HIGH);
		delay(100);
		// https://learn.adafruit.com/adafruit-esp32-s2-tft-feather/built-in-tft
		pinMode(TFT_BACKLITE, OUTPUT);
		digitalWrite(TFT_BACKLITE, HIGH);
		tft.init(TFT_HEIGHT, TFT_WIDTH); // Init ST7789 240x135
		tft.setRotation(3);
		tft.setCursor(CURSOR_X, CURSOR_Y);
		tft.fillScreen(ST77XX_BLACK);
		tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
	#endif
	tft.setTextSize(2);
	Wire.begin(); //Start I2C
	if (!rtc.begin()) {
		sprintf(paragraph, "RTC not found");
		Serial.println(paragraph); tft.println(paragraph);
	} else {
		sprintf(paragraph, "RTC found");
		Serial.println(paragraph); tft.println(paragraph);
		if (rtc.lostPower()) {
			sprintf(paragraph, "RTC time not set");
			Serial.println(paragraph); tft.println(paragraph);
			rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		}
		rtc.start();
		if (0) {
			float drift = 0; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
			float period_sec = (7 * 86400); // total obsevation period in seconds (86400 = seconds in 1 day: 7 days = (7 * 86400) seconds)
			float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
			float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
			int offset = round(deviation_ppm / drift_unit);
			rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
		} else {
			rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration
		}
		DateTime now = rtc.now();
		sprintf(datestamp, "%04d-%02d-%02d.%02d%02d%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
		Serial.println(datestamp); tft.println(datestamp);
	}
	pinMode(MICROSD_CS, OUTPUT);
	if (!sd.begin(MICROSD_CS)) {
		sprintf(paragraph, "SD card not found");
		Serial.println(paragraph); tft.println(paragraph);
		//sd.initErrorHalt(); // sometimes, this tells you that the card is bad and not to bother reformatting
	} else {
		sprintf(paragraph, "SD card found");
		Serial.println(paragraph); tft.println(paragraph);
		sprintf(paragraph, "opening log_file:");
		Serial.println(paragraph); //tft.println(paragraph);
		if (strlen(datestamp)) {
			sprintf(paragraph, "%s.%s", datestamp, log_file_name);
		} else {
			sprintf(paragraph, "%s", log_file_name);
		}
		Serial.println(paragraph); //tft.println(paragraph);
		log_file.open(paragraph, O_WRONLY | O_CREAT);
		if (log_file) {
			log_file.seek(log_file.size());
			sprintf(paragraph, "opened file for writing");
			Serial.println(paragraph);
			log_file.println(paragraph);
			log_file.sync();
		} else {
			sprintf(paragraph, "couldn't open file for writing");
			Serial.println(paragraph);
		}
		lf("/");
	}
	#ifdef USE_LORA
		setup_lora();
		if (lora_is_available) {
			//send_lora_string("coming online"); // "node4[1] coming online"
			//send_lora_string("ed209");
			//send_lora_string("robo");
		}
	#endif
	while (myGNSS.begin() == false) { //Connect to the Ublox module using Wire port
		sprintf(paragraph, "GPS not found");
		Serial.println(paragraph); tft.println(paragraph);
		delay(2000);
	}
	sprintf(paragraph, "GPS connected");
	Serial.println(paragraph); tft.println(paragraph);
	myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
	//myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Be sure RTCM3 input is enabled when using ntrip caster over wifi. UBX + RTCM3 is not a valid state.
	myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA); // UBX + RTCM3 is not a valid state.
	myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible
	myGNSS.setNavigationFrequency(NAVIGATION_FREQUENCY_HZ); //Set output in Hz.
	myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP); // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
	myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C, 10); // Tell the module to output GGA every 10 seconds
	myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed
	// from Example8_GetSetPortSettings:
	bool response;
	uint32_t currentUART2Baud = myGNSS.getVal32(UBLOX_CFG_UART2_BAUDRATE);
	sprintf(paragraph, "UART2Baud: %d", currentUART2Baud);
	Serial.println(paragraph); //tft.println(paragraph);
	if (currentUART2Baud != 57600) {
		response = myGNSS.setVal32(UBLOX_CFG_UART2_BAUDRATE, 57600);
		if (response == false) {
			sprintf(paragraph, "set baudrate fail");
		} else {
			sprintf(paragraph, "set baudrate okay");
		}
		Serial.println(paragraph); //tft.println(paragraph);
//	} else {
//		Serial.println("No baud change needed");
	}
	sprintf(paragraph, "enable uart2 RTCM3");
	Serial.println(paragraph); //tft.println(paragraph);
	response = myGNSS.setVal8(UBLOX_CFG_UART2INPROT_RTCM3X, 1); // Enable RTCM on UART2 Input
	if (response == false) {
		sprintf(paragraph, "uart2/RTCM3 fail");
	} else {
		sprintf(paragraph, "uart2/RTCM3 okay");
	}
	Serial.println(paragraph); //tft.println(paragraph);
	//myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save the ioPort and message settings to NVM
	while (Serial.available()) { // Empty the serial buffer
		Serial.read();
	}
	pinMode(BUTTON1, INPUT_PULLDOWN);
	pinMode(BUTTON2, INPUT_PULLDOWN);
	previous_button1 = digitalRead(BUTTON1);
	previous_button2 = digitalRead(BUTTON2);
	previous_button1_change_time = millis();
	previous_button2_change_time = millis();
	pingPongTime = millis();
	screenUpdateTime = millis();
	uploadTime = millis();
}

bool should_do_a_lora_pingpong = false;
bool should_do_an_upload = false;
bool should_do_a_logfile_write = false;
void loop() {
	currentTime = millis();
	//Serial.print("currentTime: ");
	//Serial.println(currentTime);
	bool button1 = digitalRead(BUTTON1);
	bool button2 = digitalRead(BUTTON2);
	bool button1_was_just_pressed = false;
	bool button2_was_just_pressed = false;
	if (previous_button1!=button1) {
		if (previous_button1_change_time<currentTime-BUTTON_DEBOUNCE_TIME) {
			if (button1) {
				//Serial.println("button1 was just pressed");
				button1_was_just_pressed = true;
			} else {
				//Serial.println("button1 was just released");
			}
			previous_button1_change_time = currentTime;
		}
		previous_button1 = button1;
	}
	if (previous_button2!=button2) {
		if (previous_button2_change_time<currentTime-BUTTON_DEBOUNCE_TIME) {
			if (button2) {
				//Serial.println("button2 was just pressed");
				button2_was_just_pressed = true;
			} else {
				//Serial.println("button2 was just released");
			}
			previous_button2_change_time = currentTime;
		}
		previous_button2 = button2;
	}
	if (button2_was_just_pressed) {
		should_do_a_lora_pingpong = true;
		//should_do_a_logfile_write = true;
	}
	if (button1_was_just_pressed) {
	}
	static short int count = 0;
	myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
	myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.
	if (LORA_PING_PONG_TIMEOUT_IN_MILLISECONDS<=currentTime-pingPongTime && should_do_a_lora_pingpong) {
		#ifdef ACQUIRE_LORA_RSSI_DATA
			if (lora_is_available) {
				send_lora_ping();
				get_lora_pong();
				pingPongTime = millis();
				should_do_a_lora_pingpong = false;
				if (lora_hAcc_mm<MINIMUM_HORIZONTAL_ACCURACY_MM) {
					should_do_a_logfile_write = true;
					should_do_an_upload = true;
				}
			}
		#endif
	} else if (SCREEN_UPDATE_TIMEOUT_IN_MILLISECONDS<=currentTime-screenUpdateTime) {
		screenUpdateTime = millis();
		sprintf(line[0], "h/v_mm: %u/%u", hAcc_mm, vAcc_mm); //tft.println(line[0]);
		sprintf(line[1], "numSV: %d %c", numSV, diffSoln?'d':' '); //tft.println(line[2]);
		//sprintf(line[5], "diffSoln: %d", diffSoln); //tft.println(line[5]);
		//sprintf(line[], "pDOP: %-*d", LENGTH_OF_LINE, pDOP); //tft.println(line[]);
		sprintf(line[2], "fixType: %-*s", LENGTH_OF_LINE, fixTypeString[fixType].c_str()); //tft.println(line[3]);
		sprintf(line[3], "carrSoln: %-*s", LENGTH_OF_LINE, carrSolnString[carrSoln].c_str()); //tft.println(line[4]);
		//snprintf(line[], "height_mm: %d", height_mm); //tft.println(line[]);
		sprintf(line[4], "p/p: %d/%d", total_number_of_pings_sent, total_number_of_pongs_received); //tft.println(line[5]);
		sprintf(line[5], "f/u/m: %d/%d/%d", total_number_of_datapoints_in_file, total_number_of_uploads, number_of_uploads_for_the_current_minute); //tft.println(line[5]);
		sprintf(line[6], "last loraRSSI: %d", lora_rssi_ping); //tft.println(line[6]);
		sprintf(line[7], "uptime: %'ld", (millis()-startTime)/1000);
		int k = 0;
		for (int l=0; l<NUMBER_OF_LINES; l++) {
			bool junk = false;
			for (int j=0; j<LENGTH_OF_LINE-1; j++, k++) {
				if (0==line[l][j]) {
					junk = true;
				}
				if (not junk) {
					paragraph[k] = line[l][j];
				} else {
					paragraph[k] = 32;
				}
			}
		}
		paragraph[k] = 0;
		//Serial.println(strnlen(paragraph, NUMBER_OF_LINES*LENGTH_OF_LINE));
		//Serial.println(paragraph);
		tft.setCursor(CURSOR_X, CURSOR_Y);
		tft.print(paragraph);
	} else if (UPLOAD_TIMEOUT_IN_MILLISECONDS<=currentTime-uploadTime) {
		if (should_do_an_upload) {
			bool okay_to_upload = false;
			if (total_number_of_uploads<MAX_UPLOADS) {
				if (number_of_uploads_for_the_current_minute<MAX_UPLOAD_RATE_PER_MINUTE) {
					okay_to_upload = true;
				}
			}
			#ifdef POST_LORA_RSSI_DATA_OVER_LORA
			if (lora_is_available) {
				#ifdef DEBUG_LORA_RSSI
					total_number_of_uploads++;
					if (0==count%1024) {
						if (lora_is_available) {
							if (RSSI_THRESHOLD<lora_rssi_ping) {
								delay(500);
								send_lora_int(lora_rssi_ping, "lora-rssi-ping");
							}
//							if (RSSI_THRESHOLD<lora_rssi_pong) {
//								delay(2000);
//								send_lora_int(lora_rssi_pong, "lora-rssi-pong");
//							}
						}
					}
				#endif
			}
			#endif
			if (okay_to_upload) {
				#ifdef POST_LORA_RSSI_DATA_OVER_LORA
					if (lora_hAcc_mm<MINIMUM_HORIZONTAL_ACCURACY_MM) {
						if (lora_is_available) {
							#ifdef DEBUG_LORA_RSSI
								delay(2000);
							#endif
							if (RSSI_THRESHOLD<lora_rssi_ping) {
								total_number_of_uploads++;
								number_of_uploads_for_the_current_minute++;
								//delay(500);
								uploadTime = millis();
								send_lora_int_with_location(lora_rssi_ping, lora_lat, lora_lon, lora_ele, "lora-rssi-ping");
								should_do_an_upload = false;
							}
//							if (RSSI_THRESHOLD<lora_rssi_pong) {
//								delay(2000);
//								send_lora_int_with_location(lora_rssi_pong, lat, lon, ele, "lora-rssi-pong");
//							}
						}
					}
				#endif
			}
		}
	}
	if (should_do_a_logfile_write) {
		if (log_file) {
			DateTime now = rtc.now();
			sprintf(datestamp, "%04d-%02d-%02d.%02d%02d%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
			if (strlen(datestamp)) {
				sprintf(paragraph, "%s, %.8f, %.8f, %.3f, %d", datestamp, lora_lat, lora_lon, lora_ele, lora_rssi_ping);
			} else {
				sprintf(paragraph, "%.8f, %.8f, %.3f, %d", lora_lat, lora_lon, lora_ele, lora_rssi_ping);
			}
			Serial.println(paragraph);
			log_file.println(paragraph);
			log_file.sync();
			total_number_of_datapoints_in_file++;
			should_do_a_logfile_write = false;
		} else {
			sprintf(paragraph, "file is not open for writing");
			Serial.println(paragraph);
		}
		lf("/");
	}
	delay(1);
	count++;
}

#ifdef USE_LORA

bool setup_lora(void) {
	if (!lora.init()) {
		sprintf(paragraph, "LoRa init failed");
		Serial.println(paragraph); tft.println(paragraph);
	} else {
		sprintf(paragraph, "LoRa init OK!");
		Serial.println(paragraph); tft.println(paragraph);
		if (!lora.setFrequency(LORA_FREQ)) {
			sprintf(paragraph, "LoRa set freq failed");
			Serial.println(paragraph); tft.println(paragraph);
		} else {
			sprintf(paragraph, "LoRa set freq OK!");
			Serial.println(paragraph); tft.println(paragraph);
		}
		sprintf(paragraph, "LoRa freq: %.1f MHz", LORA_FREQ);
		Serial.println(paragraph); tft.print(paragraph);
		lora.setTxPower(LORA_TX_POWER, false);
		//int tx_power_dbm = lora.getTxPower();
		int tx_power_dbm = LORA_TX_POWER;
		sprintf(paragraph, "LoRa TXpower: %d dBm", tx_power_dbm);
		Serial.println(paragraph); tft.print(paragraph);
		lora_is_available = true;
	}
	return lora_is_available;
}

bool send_lora_string(const char *string) {
	static unsigned messageid = 1;
	if (MAX_STRING_LENGTH<strlen(string)+9) {
		warning("send_lora_string is too long");
	}
	sprintf(sendpacket1, "node%d[%d] %s", NODEID, messageid++, string);
	if (MAX_STRING_LENGTH<strlen(sendpacket1)) {
		warning("send_lora_string is too long");
	}
	Serial.print("sending over lora: "); Serial.println(sendpacket1);
	sprintf(sendpacket2, "%s%s%s", PREFIX, sendpacket1, SUFFIX);
	int packet_length = strlen(sendpacket2);
	if (MAX_STRING_LENGTH<packet_length) {
		warning("send_lora_string is too long");
		packet_length = MAX_STRING_LENGTH;
	}
	//debug("send_lora_string(waitPacketSent)"); // sometimes it hangs after this line
	lora.waitPacketSent(); // wait for the previous thing to finish sending
	//delay(100);
	//debug("send_lora_string(send)"); // other times it hangs after this line
	lora.send((uint8_t*) sendpacket2, packet_length); // returns immediately, but message takes another 70 ms to get sent
	//debug("send_lora_string(return)");
	return true;
}

bool send_lora_float_with_location(float value, float latitude, float longitude, float elevation, const char *string) {
	snprintf(sendpacket3, MAX_STRING_LENGTH, "%s [%f,%.8f,%.8f,%.3f]", string, value, latitude, longitude, elevation);
	return send_lora_string(sendpacket3);
}

bool send_lora_int_with_location(int value, float latitude, float longitude, float elevation, const char *string) {
	snprintf(sendpacket3, MAX_STRING_LENGTH, "%s [%d,%.8f,%.8f,%.3f]", string, value, latitude, longitude, elevation);
	return send_lora_string(sendpacket3);
}

bool send_lora_float(float value, const char *string) {
	snprintf(sendpacket3, MAX_STRING_LENGTH, "%s [%f]", string, value);
	return send_lora_string(sendpacket3);
}

bool send_lora_int(int value, const char *string) {
	snprintf(sendpacket3, MAX_STRING_LENGTH, "%s [%d]", string, value);
	return send_lora_string(sendpacket3);
}

int get_lora_rssi(void) {
	int rssi = lora.lastRssi();
	return rssi;
}

bool parse_lora_raw(const char *raw_message) {
	//debug("parse_lora_raw()");
	// compare to regexp: re.search("^" + PREFIX + "node([0-9]+)\[([0-9]+)\](.*)" + SUFFIX + "$", packet_text)
	int len = strnlen(raw_message, MAX_STRING_LENGTH);
	//Serial.print("received lora string length: "); Serial.println(len);
	//Serial.println(raw_message);
//	int section = 0;
	int p = strlen(PREFIX);
	int s = strlen(SUFFIX);
	// "SCOOPYnode1[185] pong rssi=-101BOOPS"
	if (len<p+s) {
		Serial.println("message too short for prefix+suffix");
		return false;
	}
	int i = 0;
	int j = 0;
	for (i=0, j=0; i<p && j<p; i++, j++) {
		if (raw_message[i]!=PREFIX[j]) {
			Serial.print("message does not match prefix at index ");
			Serial.print(i);
			Serial.print(",");
			Serial.println(j);
			return false;
		}
	}
	for (i=len-1, j=s-1; i<=len-s && j<=0; i--, j--) {
		if (raw_message[i]!=SUFFIX[j]) {
			Serial.print("message does not match suffix at index ");
			Serial.print(i);
			Serial.print(",");
			Serial.println(j);
			return false;
		}
	}
	const char NODE[] = "node";
	int string_len = strlen(NODE);
	for (i=p, j=0; i<p+string_len-1 && j<string_len; i++, j++) { // i off-by-frog
		if (raw_message[i]!=NODE[j]) {
			Serial.print("message does not match \"node\" at index ");
			Serial.print(i);
			Serial.print(",");
			Serial.println(j);
			return false;
		}
	}
	i++; // i off-by-frog
	for (; i<len-s; i++) {
		bool match = false;
		for (j=0; j<10; j++) {
			if (raw_message[i]=='0'+j) {
				match = true;
			} else if (raw_message[i]=='[') {
				goto next1;
			}
		}
		if (!match) {
			Serial.print("message does not match [0-9] at index ");
			Serial.println(i);
			return false;
		}
	}
next1:
	if (raw_message[i]!='[') {
		Serial.print("message does not match [ at index ");
		Serial.println(i);
		return false;
	}
	i++;
	for (; i<len-s; i++) {
		bool match = false;
		for (j=0; j<10; j++) {
			if (raw_message[i]=='0'+j) {
				match = true;
			} else if (raw_message[i]==']') {
				goto next2;
			}
		}
		if (!match) {
			Serial.print("message does not match [0-9] at index ");
			Serial.println(i);
			return false;
		}
	}
next2:
	if (raw_message[i]!=']') {
		Serial.print("message does not match ] at index ");
		Serial.println(i);
		return false;
	}
	i++;
	if (raw_message[i]!=' ') {
		Serial.print("message does not match <space> at index ");
		Serial.println(i);
		return false;
	}
	i++;
	strncpy(received_message, raw_message+i, len-s-i);
	received_message[len-s-i] = 0;
	//Serial.print("remaining message: ");
	//Serial.println(message);
	//debug("parse_lora_raw(return)");
	return true;
}

bool parse_lora_message(const char *string) {
	//debug("parse_lora_message()");
	// "pong rssi=-101"
	int string_len = strlen(string);
	int len = strnlen(received_message, MAX_STRING_LENGTH);
	int i = 0;
	int j = 0;
	for (i=0; i<len && j<string_len; i++, j++) {
		if (received_message[i]!=string[j]) {
			Serial.print("message does not match ");
			Serial.print(string);
			Serial.print(" at index ");
			Serial.print(i);
			Serial.print(",");
			Serial.println(j);
			return false;
		}
	}
	const char RSSI[] = " rssi=-";
	string_len = strlen(RSSI);
	for (j=0; i<len && j<string_len; i++, j++) {
		if (received_message[i]!=RSSI[j]) {
			Serial.print("message does not match \" rssi=-\" at index ");
			Serial.print(i);
			Serial.print(",");
			Serial.println(j);
			return false;
		}
	}
	int index_a = i;
	for (; i<len; i++) {
		bool match = false;
		for (j=0; j<10; j++) {
			if (received_message[i]=='0'+j) {
				match = true;
			}
		}
		if (!match) {
			Serial.print("message does not match [0-9] at index ");
			Serial.println(i);
			return false;
		}
	}
	int index_b = i;
	char numeric_string[MAX_STRING_LENGTH];
	strncpy(numeric_string, received_message+index_a, index_b-index_a);
	numeric_string[index_b-index_a] = 0;
	lora_rssi_ping = -atoi(numeric_string);
	total_number_of_pongs_received++;
	lora_hAcc_mm = hAcc_mm;
	lora_lat = lat;
	lora_lon = lon;
	lora_ele = ele;
	//debug("parse_lora_message(return)");
	return true;
}

void send_lora_ping(void) {
	lora_rssi_ping = JUNK_RSSI;
	lora_rssi_pong = JUNK_RSSI;
	lora_hAcc_mm = JUNK_ACC_MM;
	lora_lat = JUNK_LAT;
	lora_lon = JUNK_LON;
	lora_ele = JUNK_ELE;
	send_lora_string("ping");
	total_number_of_pings_sent++;
}

void get_lora_pong(void) {
	//debug("get_lora_pong(waitPacketSent)");
	lora.waitPacketSent(); // wait for the previous thing to finish sending
	if (lora.waitAvailableTimeout(500)) { // only waits the full time if there's nothing
		//debug("get_lora_pong(waitAvailableTimeout())");
		uint8_t len = MAX_STRING_LENGTH;
		lora.recv(recvpacket, &len);
		//debug("get_lora_pong(recv)");
		if (0<len) {
			recvpacket[len] = 0;
			if (parse_lora_raw((const char *) recvpacket)) {
				if (parse_lora_message("pong")) {
					Serial.print("response: ");
					Serial.println(received_message);
					lora_rssi_pong = lora.lastRssi();
					//Serial.print("response rssi: ");
					//Serial.println(lora_rssi_pong);
				} else {
					Serial.print("response was not pong: \"");
					Serial.print(received_message);
					Serial.println("\"");
				}
			}
		} else {
			Serial.println("received packet was length 0");
		}
	} else {
		Serial.println("did not receive a response");
	}
	//debug("get_lora_pong(return)");
}

#endif

