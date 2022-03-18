//
//  Config.h
//
//  Hardware configuration, some hardcoded parameters and default values
//

//
// Name of this project, initial device name
//  * copied to *Device_name used later on, and unless configured,
//  * used as the local/dhcp hostname
//
#define DEVICE_NAME "NTP-clock"

//
// Verbose time display related extra debug to serial console
//
#undef DEBUG_TIME_DISPLAY

//
// Web server initial defaults
//
#define HTTPD_PORT  80
#define HTTPD_LOGIN "admin"
#define HTTPD_PASS  "ntpclock" // unless using the chip id
#undef  HTTPD_PASS_USE_CHIP_ID // use chip id in upper case hex digits as default password ("01A2B3")

//
// For dealing with NTP & the clock (initial defaults)
//
#define CLOCK_DRIFT_CORRECTION 0.0 // initial clock drift correction value, ms per hour
#define NTP_UPDATE_FREQ 3600 // s, starts at 1/divisor (see NTPClient _updateDivisor)
#define NTP_SERVER "europe.pool.ntp.org"
// UTC offset in hours (float, to support weird zones)
#define TIMEZONE 2.0
// To apply European Union Daylight Saving Time (DST), set NTP_APPLY_EU_DST true
#define NTP_APPLY_EU_DST 1

//
// SPIFFS path to the non-volatile configuration
//
#define MAIN_CONFIG_FILE "ntp-clock.conf" // no paths in SPIFFS since 2.5

// Default display mode setting: seconds and fractions = 0, date = 1 (Quiet_mode)
#define DEFAULT_DISPLAY_MODE 1
#define DEFAULT_BRIGHTNESS   3 // 0 = OFF, 1 = min (not very dim), 8 max (VERY bright)
#define DP1_BRIGHTNESS_TABLE { 0, 1, 2, 3, 4, 5, 6, 7, 8 } // Actual setting. DP1 should probably just be linear.
#define DP2_BRIGHTNESS_TABLE { 0, 1, 2, 3, 4, 5, 6, 7, 8 }
#define MTX_BRIGHTNESS_TABLE { 0, 1, 1, 2, 2, 3, 3, 4, 5 }

// Default delays for displaying stuff
#define DISPLAY_DELAY_1  333  // ms, very short
#define DISPLAY_DELAY_2  666  // ms, short
#define DISPLAY_DELAY_3 1000  // ms, medium, for IP address display

// Define NMEA for USBserial GNSS input code (parses $GxRMC sentences)
// (Currently only developed for a dual display + hwswitch unit)
#undef  HAVE_NMEA
#undef  DEBUG_NMEA_RMC // Echo parsed values

//
// Hardware configuration
// DO NOT change these defaults here, use a hardware config 
// block with necessary #undef's and #defines later on to 
// create a device specific configuration!
//
#define HAVE_DP1
#undef  HAVE_DP2
#undef  HAVE_MATRIX
#undef  HAVE_GY49
#undef  HAVE_SWITCH
#undef  HAVE_INTLED
//
// Type of 7-segment displays
// leave undefined for 'XX:XX', define for 'X.X.X.X.'
//
#undef  DP1_DECIMAL
#undef  DP2_DECIMAL
//
// Matrix display weekday locale, choose one
//
#undef  WEEKDAYS_LANG_EN
#define WEEKDAYS_LANG_FI
//
// Pin/address definitions for GY-49 MAX44009, TM1637, matrix, h/w switch
// Most can be changed to other ports, but GY-49 needs SCL on D1 and SDA on D2.
// D8 is works well for a hardware switch (normally open), short to 3.3V.
// On a Wemos D1 mini, the internal LED shares D4, works OK shared as DIO for 
// a display; Recommend using it for display 1 if you need ALL of the pins.
//
#define GY49_ADDR 0x4A
#define DP1_CLK D0
#define DP1_DIO D6
#define DP2_CLK D3
#define DP2_DIO D4
#define MTX_CLK D5
#define MTX_DIO D7
#define SWINPUT D8

//
// Hardware configuration shortcut blocks
//

// at home/OH, home/MH
#if defined(JK_HOME_1)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-1"
  #undef  NTP_SERVER
  #define NTP_SERVER "192.168.1.1"
  #undef  DP1_CLK
  #undef  DP1_DIO
  #define DP1_CLK D3
  #define DP1_DIO D2
  #undef  HAVE_SWITCH

// at home/jk
#elif defined(JK_HOME_2MGS)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-jk"
  #undef  NTP_SERVER
  #define NTP_SERVER "192.168.1.1"
  #define HAVE_DP2
  #define HAVE_MATRIX
  #define HAVE_GY49
  #define HAVE_SWITCH
  #undef  MTX_BRIGHTNESS_TABLE
  #define MTX_BRIGHTNESS_TABLE { 0, 1, 2, 3, 4, 5, 6, 7, 8 }

// at home/travel, NMEA input ready, 2x blue clock displays
#elif defined(JK_NMEA_2GS)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-blue"
  #undef  NTP_SERVER
  #define NTP_SERVER "ntp.aalto.fi"
  #define HAVE_DP2
  #define DP2_DECIMAL
  #undef  POINT_DATE
  #define POINT_DATE POINT_OFF
  #define HAVE_INTLED
  #define HAVE_GY49
  #define HAVE_SWITCH
  #define HAVE_NMEA

// at home/travel, 2x blue decimal displays
#elif defined(JK_PRO_2GS)
  #undef DP1_CLK
  #undef DP1_DIO
  #undef DP2_CLK
  #undef DP2_DIO
  #define DP1_CLK D0
  #define DP1_DIO D5
  #define DP2_CLK D6
  #define DP2_DIO D7
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-pro"
  #undef  NTP_SERVER
  #define NTP_SERVER "192.168.1.1"
  #define HAVE_DP2
  #define DP1_DECIMAL
  #define DP2_DECIMAL
  #define HAVE_INTLED
  #define HAVE_GY49
  #define HAVE_SWITCH

// at Aalto/jk
#elif defined(JK_AALTO_2MGS)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTPclockJK2"
  #undef  NTP_SERVER
  #define NTP_SERVER "ntp.aalto.fi"
  #define DP1_DECIMAL
  #define DP2_DECIMAL
  #define HAVE_DP2
  #define HAVE_MATRIX
  #define HAVE_GY49
  #define HAVE_SWITCH

// Aalto clock at home
#elif defined(JK_OH_2MGS)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-jk"
  #undef  NTP_SERVER
  #define NTP_SERVER "192.168.1.1"
  #define DP1_DECIMAL
  #define DP2_DECIMAL
  #define HAVE_DP2
  #define HAVE_MATRIX
  #define HAVE_GY49
  #define HAVE_SWITCH
  #undef  DP2_BRIGHTNESS_TABLE
  #define DP2_BRIGHTNESS_TABLE { 0, 1, 1, 2, 3, 3, 4, 5, 8 }
  #undef  MTX_BRIGHTNESS_TABLE
  #define MTX_BRIGHTNESS_TABLE { 0, 0, 1, 1, 2, 3, 3, 3, 4 }

// at boat, NMEA input, with matrix
#elif defined(JK_NMEA_2MGS)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-N2"
  #undef  NTP_SERVER
  #define NTP_SERVER "ntp.aalto.fi"
  #define DP1_DECIMAL
  #define DP2_DECIMAL
  #define HAVE_DP1
  #define HAVE_DP2
  #define HAVE_MATRIX
  #define HAVE_GY49
  #define HAVE_SWITCH
  #define HAVE_NMEA
  #undef  MTX_BRIGHTNESS_TABLE
  #define MTX_BRIGHTNESS_TABLE { 0, 1, 1, 1, 2, 2, 3, 3, 4 }

// at Aalto, headless debug board
#elif defined(JK_AALTO_ND)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTPclockJK0"
  #undef  NTP_SERVER
  #define NTP_SERVER "ntp.aalto.fi"
  #undef  HAVE_DP1
  #define HAVE_SWITCH
  #define HAVE_INTLED
  #define HAVE_NMEA
  #define DEBUG_NMEA_RMC

// at TJ
#elif defined(JK_TJ_2GS)
  #undef  NTP_SERVER
  #define NTP_SERVER "ntp.aalto.fi"
  #undef  DP2_CLK
  #undef  DP2_DIO
  #define DP2_CLK D5
  #define DP2_DIO D7
  #define DP1_DECIMAL
  #define DP2_DECIMAL
  #define HAVE_DP2
  #define HAVE_GY49
  #define HAVE_SWITCH
  
// at TJ
#elif defined(JK_TJ_2MGS)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-TJ2"
  #undef  NTP_SERVER
  #define NTP_SERVER "ntp.aalto.fi"
  #define DP1_DECIMAL
  #define DP2_DECIMAL
  #define HAVE_DP1
  #define HAVE_DP2
  #define HAVE_MATRIX
  #define HAVE_GY49
  #define HAVE_SWITCH
  #undef  HAVE_NMEA
  #undef  DP2_BRIGHTNESS_TABLE
  #define DP2_BRIGHTNESS_TABLE { 0, 1, 1, 2, 3, 3, 4, 5, 8 }
  #undef  MTX_BRIGHTNESS_TABLE
  #define MTX_BRIGHTNESS_TABLE { 0, 1, 1, 1, 2, 3, 3, 3, 4 }

// at TJ
#elif defined(JK_FERNIS_2MGS)
  #undef  DEVICE_NAME
  #define DEVICE_NAME "NTP-clock-Fernis"
  #undef  NTP_SERVER
  #define NTP_SERVER "ntp.aalto.fi"
  #define DP1_DECIMAL
  #define DP2_DECIMAL
  #define HAVE_DP1
  #define HAVE_DP2
  #define HAVE_MATRIX
  #define HAVE_GY49
  #define HAVE_SWITCH
  #undef  HAVE_NMEA
  #undef  DP2_BRIGHTNESS_TABLE
  #define DP2_BRIGHTNESS_TABLE { 0, 1, 1, 2, 3, 3, 4, 5, 8 }
  #undef  MTX_BRIGHTNESS_TABLE
  #define MTX_BRIGHTNESS_TABLE { 0, 1, 1, 1, 2, 3, 3, 3, 4 }
#endif
