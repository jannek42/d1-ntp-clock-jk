//
//  D1_ntp_clock_jk.h
//
//  Other definitions, main program globals & prototypes
//
#define MASTER_VERSION "2021-10-12.1"
#define DEBUG_PARSE_CONFIG_FILE false
#define NO_AUTH_IF_DEFAULT_PASS true
#pragma GCC diagnostic ignored "-Wformat-truncation"

//
//  Display globals (and prerequisites for defining more globals later)
//
#ifdef HAVE_DP1
  TM1637 display1(DP1_CLK, DP1_DIO);
#else
  void *display1;
#endif
#ifdef HAVE_DP2
  TM1637 display2(DP2_CLK, DP2_DIO);
#else
  void *display2;
#endif
#ifdef HAVE_MATRIX
  MLEDScroll matrix(1, MTX_DIO, MTX_CLK, false);
#else
  void *matrix;
#endif

// The mid point of 7-segment displays run by TM1637 may or may not 
// be the same for '88:88' and '8.8.8.8.' type displays. 
// They _seem_ to match, but just light 'em all up for clock displays.
#ifdef DP1_DECIMAL
  #define DP1_POINT_CLOCK POINT_TIME
#else
  #define DP1_POINT_CLOCK POINT_ALL
#endif
#ifdef DP2_DECIMAL
  #define DP2_POINT_CLOCK POINT_TIME
  #define DATEDISP_FORMAT "%2d%2d"
#else
  #define DP2_POINT_CLOCK POINT_ALL
  //#define DATEDISP_FORMAT "%02d%02d"
  #define DATEDISP_FORMAT "%2d%2d"
#endif

// Internal LED state
#define ON  LOW
#define OFF HIGH

// Matrix display weekday name icon offset
#define MTX_WKD_OFFSET_EN 25 // see MLEDScrollFonts.h
#define MTX_WKD_OFFSET_FI 18 // see MLEDScrollFonts.h
#ifdef WEEKDAYS_LANG_EN
  #define MTX_WKD_OFFSET MTX_WKD_OFFSET_EN
  #define MTX_WKD_LANG 0
#endif
#ifdef WEEKDAYS_LANG_FI
  #define MTX_WKD_OFFSET MTX_WKD_OFFSET_FI
  #define MTX_WKD_LANG 1
#endif
// Matrix display symbols
#define MTX_ICON_SOG 245
#define MTX_ICON_COG 244
#define MTX_ICON_SOG_COG 251
#define MTX_ICON_COG_SOG 248

// Matrix display analog seconds icon offset
#define MTX_SECMAP_OFFSET 97 // see MLEDScrollFonts.h

// NMEA timeout, millis, other NMEA stuff
#define NMEA_TIMEOUT 5000.0
#define KN_TO_KMH     1.852
#define KN_TO_MS      0.514444
#define SOG_UNIT_KN   0
#define SOG_UNIT_KMH  1
#define SOG_UNIT_MS   2

// char[MAX] for some global pointers and other stuff
#define MAX_NTP_SERVER_STR  256
#define MAX_HTTPD_LOGIN_STR  32
#define MAX_DEVICE_NAME_STR  32
#define MAX_AP_STR            9 // DO NOT CHANGE. See wifi callback.
//#define MAX_HTTPD_OUT      1700
#define MAX_HTTPD_OUT      2000
#define MAX_HTTPD_SS        256
#define MAX_SERIAL_INPUT   1024

//
// Night mode settings
//
#define NIGHT_MODE_OFF  0
#define NIGHT_MODE_ON   1
#define NIGHT_MODE_AUTO 2

//
// NTP client globals and UDP socket it uses.
//
float      Timezone = TIMEZONE; // in hours, +/- from UTC
unsigned long Ntp_update_freq = NTP_UPDATE_FREQ; // NTP poll frequency, s
double     Clock_drift_comp = CLOCK_DRIFT_CORRECTION; // +/- ms/h
int        Ntp_apply_eu_dst = NTP_APPLY_EU_DST; // 1 to apply EU DST rules, 0 to not
char      *Ntp_server; // char[MAX_NTP_SERVER_STR] allocated in setup()
WiFiUDP    Ntp_udp;
NTPClient *TimeClient; // Allocated in setup()

//
// ESP chip id, friendly device name
//
unsigned int Chip_id;
char *Device_name; // Friendly device name - char[MAX_DEVICE_NAME_STR] allocated in setup()

//
// Initial network configuration access point name and password
//
char *Cfg_ap_ssid; // char[MAX_AP_STR] allocated in setup()
char *Cfg_ap_pass; // char[MAX_AP_STR] allocated in setup()

//
// Web server globals
//
ESP8266WebServer Httpd(HTTPD_PORT);
char *Webserver_login; // char[MAX_HTTPD_LOGIN_STR] allocated in setup()
char *Webserver_pass;  // char[MAX_HTTPD_LOGIN_STR] allocated in setup()
#define MIME_TEXT "text/plain"
#define MIME_HTML "text/html"

//
// Other globals
//
bool    Have_nmea     = false; // NMEA functionality enabled
bool    Have_dp[]     = { false, false, false }; // 4x7-seg display n available (ignore index 0)
bool    Have_matrix   = false; // Matrix display available
bool    Have_gy49     = false; // GY-49 light sensor available
bool    Have_switch   = false; // Button switch available
bool    Have_intled   = false; // Use D1 internal LED
uint8_t Have_displays = 0; // Defined later, number of 7-segment displays
uint8_t Altdisplay    = 0; // Defined later, selected secondary display
uint8_t Dp[]          = { 0, 1, 2 }; // Display id's (enable swapping, ignore Dp[0])
uint8_t DpPc[]        = { POINT_OFF, DP1_POINT_CLOCK, DP2_POINT_CLOCK };
bool    Swap_displays = false; // 
bool    Decimal_dp[]  = { false, false, false }; // 4x7 display n has decimal points (ignore index 0)
double  Lux_step[]    = { 0.0f, 0.5f, 2.0f, 10.0f, 20.0f, 30.0f, 200.0f, 500.0f }; // Default brightness level switching steps, lux
int     Dp_bright[3][9] = { MTX_BRIGHTNESS_TABLE, DP1_BRIGHTNESS_TABLE, DP2_BRIGHTNESS_TABLE } ; // Secondary 4x7 brightness level vs. primary 4x7 level, usually linear.
int    *Mtx_bright    = Dp_bright[0];
int    *Dp1_bright    = Dp_bright[1];
int    *Dp2_bright    = Dp_bright[2];
uint8_t Night_mode    = NIGHT_MODE_OFF; // no-blink minute change and alternate, fixed brightness levels
uint8_t Prev_nm       = NIGHT_MODE_OFF; // prev night mode (if !quiet while in nm)
uint8_t Night_extreme = 0; // Lose the minutes.
uint32_t NiEx_h       = 0; // Which hours to activate extreme night mode on
uint8_t NiEx_from     = 22; // convenience globals for the former...
uint8_t NiEx_to       = 8;  // because lazy.
const char *Night_mode_str[] = { "Off", "On", "Auto" };
int     Night_bright[] = { 0, 1, 0 }; // Night mode brightness, matrix/dp1/dp2
uint8_t Night_swap    = 0; // Swap primary/secondary display at night
double  Night_lux     = 0.0f; // Night mode triggered at lux <= x
const char *Night_swap_str[] = { "Disable", "Enable" };
const char *Night_extreme_str[] = { "Disable", "Enable" };
double  Start_time    = 0.0; // For uptime calculation
float   Luminance     = -1.0F; // Current ambient light level
int     Brightness    = DEFAULT_BRIGHTNESS; // Current brightness step
int     Manual_bright = -1; // Override if > -1. 0 == OFF.
bool    Quiet_mode    = DEFAULT_DISPLAY_MODE; // true => date, false => ms
uint8_t Show_year     = 0; // show current year for n seconds
bool    Refresh       = false; // Force refresh of all displays
int         Mtx_wd_lc     = MTX_WKD_LANG; // Default, can be configured later
const int   Mtx_wd_idx[]  = { MTX_WKD_OFFSET_EN, MTX_WKD_OFFSET_FI }; // lookup table
const char *Mtx_wd_lang[] = { "English", "Finnish" }; // lookup table
uint8_t H_lead_zero   = 0; // Use leading zero for single digit hours, 1 = yes, 0 = no
const char *Hlzero_str[] = { "Disable", "Enable" };
const char *Time_format[] = { "%2d%02d", "%02d%02d"}; // lookup table, H_lead_zero as index
const char *Time_format_niex[] = { "%2d", "%02d"};    // lookup table, H_lead_zero as index, extreme night mode
const char *Sog_unit_str[] = { "kn", "km/h", "m/s" }; // Speed over ground unit: 0=kn, 1=km/h, 2=m/s
int     NMEA_mode     = 0; // NMEA display mode
int     NMEA_sog_unit = SOG_UNIT_KN; // GPS native unit
int     NMEA_init_mode = 1; // NMEA mode, when input received after silence
const char *NMEA_mode_str[] = { "Inactive", "COG/SOG", "Time/COG", "Time/SOG", "Time/Date", "Time/Sec" };
#ifdef HAVE_NMEA
unsigned long NMEA_ts = 0; // Last NMEA sentence
char    NMEA_sog[5]; // NMEA speed over ground
char    NMEA_cog[5]; // NMEA course over ground
double  NMEA_sog_d = -1.0;
double  NMEA_cog_d = -1.0;
double  Sog_multiplier[] = { 1, KN_TO_KMH, KN_TO_MS };
#endif

// Text/HTML record start, separator and end
const char *sr_a[] = { "",     "<tr><th>" };
const char *sp_a[] = { ": ",   "</th><td>" };
const char *er_a[] = { "\r\n", "</td></tr>\r\n" };

//
// WiFi status codes explained, since there's no such thing in the library..
//
#define MAX_WL_STATUS_ENUM 6
const char *WiFi_status[] = { 
  "status changing",                   // 0, WL_IDLE_STATUS
  "configured WLAN not available",     // 1, WL_NO_SSID_AVAIL
  "scan complete",                     // 2, WL_SCAN_COMPLETED 
  "connected",                         // 3, WL_CONNECTED
  "connect failed (check password)",   // 4, WL_CONNECT_FAILED
  "connection lost",                   // 5, WL_CONNECTION_LOST
  "disconnected"                       // 6, WL_DISCONNECTED
};

bool Debug_enabled = true;
bool Debug_config = DEBUG_PARSE_CONFIG_FILE;
DebugClass DebugMain;
DebugClass *Debug = &DebugMain;


//
// Prototypes
//
void setup(void);
void setup_ota(void);
void wifi_ap_callback(WiFiManager *WiFiAP);
void on_before_ntp(void);
void on_after_ntp(void);
bool load_config(void);
bool load_config(const char *cfg_path);
bool have_default_password(void);
bool need_to_authorize(void);
void parse_lux_config_str(char *str, bool reading_config);
void parse_lux_config_str(char *str);
void parse_bri_str(uint8_t dnum, char *str);
void parse_mtx_bri_str(char *str);
void parse_night_bri_str(char *str);
void update_night_extreme_mask(void);
bool save_config(void);
bool save_config(const char *cfg_path);
void debug_print_config(void);
const char *get_hwcfg_str(void);
const char *get_hwcfg_str(int html);
const char *get_timezone_str(void);
const char *get_timezone_str(int html);
const char *get_timezone_str(double epochtime);
const char *get_timezone_str(double epochtime, int html);
const char *get_uptime_str(void);
const char *get_uptime_str(unsigned long seconds);
const char *get_lux_str(void);
const char *get_lux_str(int html);
const char *get_drift_str(void);
const char *get_info_str(void);
const char *get_info_str(int html);
void send_html_header(void);
void send_html_header(const char *title);
void send_html_style(void);
void send_html_footer(void);
bool html_start(void);
bool html_start(const char *title);
void html_end(void);
void html_end(bool show_footer);
void setup_httpd(void);
bool httpd_check_auth(void);
void httpd_root(void);
void httpd_mode(void);
void httpd_swap_displays(void);
#ifdef HAVE_NMEA
void httpd_sog_unit(void);
#endif
void httpd_gettime(void);
void httpd_getlux(void);
void httpd_getinfo(void);
void httpd_reboot(void);
void httpd_ntpsync(void);
void httpd_config(void);
void httpd_options(void);
void httpd_cfgpassw(void);
void httpd_setpassw(void);
void httpd_saveconfig(void);
void httpd_setconfig(void);
void _httpd_setconfig_ss(char *ss, int *ok);
bool check_serial_input(void);
bool check_luminosity(void);
bool check_wifi_status(void);
bool check_ip_addr(void);
void display_swap(void);
void display_swap(bool reset_order);
void display_ip_addr(void);
void display_ip_addr(int header_mode);
void display_ip_addr(IPAddress ip);
void display_ip_addr(IPAddress ip, int header_mode);
void display_clear(uint8_t dnum);
void display_init(uint8_t dnum);
void display_brightness(uint8_t dnum, int bri);
void display_point(uint8_t dnum, uint8_t state);
void display_show_str(uint8_t dnum, const char *str);
void display_show_str(uint8_t dnum, const char *str, unsigned int delay_after);
void display_show_char(uint8_t dnum, uint8_t addr, unsigned char c);
void display_show_char(uint8_t dnum, uint8_t addr, unsigned char c, unsigned int delay_after);
void altdisplay_show_str(const char *str);
void altdisplay_show_str(const char *str, unsigned int delay_after);
void altdisplay_show_str(const char *str, unsigned int delay_before, unsigned int delay_after);
void intled(uint8_t state);
void intled(uint8_t state, unsigned int delay_after);
void matrix_init(void);
void matrix_clear(void);
void matrix_brightness(int bri);
void matrix_icon(uint8_t i);
void matrix_char(unsigned char c);
void set_brightness(void);
void set_brightness(int bri);
void set_night_mode(void);
void set_night_mode(uint8_t mode);
void set_night_swap(void);
void set_night_swap(uint8_t mode);
void cycle_night_mode(void);
void cycle_night_mode(int dir);
void toggle_mode(void);
#ifdef HAVE_NMEA
void parse_nmea_rmc(char *str);
void format_nmea_cog_sog(void);
#endif
void loop(void);

// EOF //
