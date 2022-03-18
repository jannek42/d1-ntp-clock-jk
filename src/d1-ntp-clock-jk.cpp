//
//   Wemos D1 mini NTP clock - Janne Korkkula <jk@iki.fi> 2018-2020
//
//   updated: 2020-09-08
//
//   TM1637 7-segment displays, one or two
//   Optional 1640 8x8 LED matrix
//   Optional GY-49 MAX44009 brightness control
//   Optional momentary switch
//   Optional GNSS $GxRMC to USBserial -> time,SOG / SOG,COG display
//
//   The idea and initial code from Steve Kemp
//   https://steve.fi/Hardware/
//

//   Serial input commands:
//   ======================
//   get info:  info, time, ip, config, lux
//   actions:   mode (toggle display mode), 
//              swap (swap displays 1 & 2 - for most output),
//              unit (switch SOG unit kn -> km/h -> m/s ... )
//              sync (force an ntp sync), 
//              pass=newpassword,
//              tz=float (set timezone),
//              poll=s (set NTP update frequency)
//              drift=ms (set clock drift compensation),
//              hlzero=0|1 (set hour leading zero off/on),
//              nite=0|1|2 (night mode, off/auto/forced),
//              nisw=0|1 (swap primary/secondary display in night mode, off/on),
//              nilx=float (night mode trigger lux)
//              nibr=n,n,n (night mode brightness levels, matrix/dp1/dp2)
//              niex=0|1 (extreme night mode on/off)
//              nixs=0-23 (start extreme night mode at h)
//              nixe=0-23 (end extreme night mode at h)
//              lang=0|1 (set matrix weekday language), 
//              bri=-1..8 (set forced brightness, 0 = OFF, -1 = auto)
//              lxst=n.n,... (set lux steps for brightness levels)
//              dp1b=n,n,... (set primary display brightness at disp.brightness levels)
//              dp2b=n,n,... (set secondary display brightness at disp.brightness levels)
//              mtxb=n,n,... (set matrix brightness at disp.brightness levels)
//              load (config file) (skips some steps though),
//              save (config file), 
//              reboot (warm reboot),
//   debug:     debug, 
//              nodebug (serial output on, off)

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include <WiFiManager.h>
#include <string.h>

#include "NTPClient.h"
#include "TM1637.h"
#include "MLEDScroll.h"
#include "Config.h"
#include "D1_ntp_clock_jk.h"
#include "Debug.h"

// #include <string.h> does not take care of this, somehow
extern char *strtok_r(char *, const char *, char **);

// Build info, defined here to get the main .ino/.cpp
const char *Build = __FILE__ " " __DATE__ " " __TIME__;

//
// setup() is called when the device is powered-on.
//
void setup(void) {
  // Enable our serial port.
  Serial.begin(115200);

  debug_log_ln("Starting...");
  if (strrchr(Build, '/'))       Build = strrchr(Build, '/')  + 1;
  else if (strrchr(Build, '\\')) Build = strrchr(Build, '\\') + 1;

  static char device_name_str[MAX_DEVICE_NAME_STR];
  snprintf(device_name_str, MAX_DEVICE_NAME_STR, "%s", DEVICE_NAME);
  Device_name = device_name_str;
  Chip_id = ESP.getChipId();

  static char cfg_ap_ssid_str[MAX_AP_STR];
  static char cfg_ap_pass_str[MAX_AP_STR];
  snprintf(cfg_ap_ssid_str, MAX_AP_STR, "NTP-%04X", Chip_id & 0xffff);
  snprintf(cfg_ap_pass_str, MAX_AP_STR, "ESP-%04d", Chip_id % 10000);
  Cfg_ap_ssid = cfg_ap_ssid_str;
  Cfg_ap_pass = cfg_ap_pass_str;
  
  // Mount FS
  LittleFS.begin();

  // Set convenience globals
  #ifdef HAVE_DP1
    Have_dp[1] = true;
    Have_displays++;
  #endif
  #ifdef HAVE_DP2
    Have_dp[2] = true;
    Have_displays++;
  #endif
  #ifdef DP1_DECIMAL
    Decimal_dp[1] = true;
  #endif
  #ifdef DP2_DECIMAL
    Decimal_dp[2] = true;
  #endif
  #ifdef HAVE_MATRIX
    Have_matrix = true;
  #endif
  #ifdef HAVE_GY49
    Have_gy49 = true;
  #endif
  #ifdef HAVE_INTLED
    Have_intled = true;
  #endif
  #ifdef HAVE_SWITCH
    Have_switch = true;
  #endif
  #ifdef HAVE_NMEA
    Have_nmea = true;
  #endif

  if (Have_displays > 1)
    Altdisplay = 2; // Select alternate display number, usually 2.
  else if (Have_displays)
    Altdisplay = 1; // Oh well, use the main display with a delay

  // Initialize the internal LED, on during setup
  if (Have_intled) {
    debug_log_ln("IntLED: init");
    pinMode(LED_BUILTIN, OUTPUT);
    intled(ON);
  }

  // Output hardware config
  debug_log(get_hwcfg_str());

  // Initialize the displays, segment test & identify
  if (Have_displays || Have_matrix) {
    debug_log_ln("Displays: init");
    set_brightness(-1); // force DEFAULT_BRIGHTNESS
  }

  if (Have_displays) debug_log_ln("Displays: test");
  for (uint8_t i=1; i<=2; i++) {
    display_point(i, (Decimal_dp[i] ? POINT_ALL : POINT_ON));
    display_show_str(i, "8888");
  }
  if (Have_matrix) {
    debug_log_ln("Matrix: init");
    matrix_init();
    matrix_char(0xff);
  }
  delay(DISPLAY_DELAY_3);

  if (Have_displays) debug_log_ln("Displays: identify");
  for (uint8_t i = 1; i <= Have_displays; i++) {
    display_point(i, POINT_OFF);
    for (uint8_t j = 0; j < 4; j++) {
      display_show_char(i, j, i);
    }
  }
  if (Have_matrix) {
    matrix_char('m');
  }
  delay(DISPLAY_DELAY_3);

  // Initialize the GY-49 ambient light sensor
  if (Have_gy49) {
    debug_log_ln("GY49: init");
    Wire.begin();
    Wire.beginTransmission(GY49_ADDR);
    Wire.write(0x02);
    Wire.write(0x40);
    Wire.endTransmission();
  }

  // Initialize the hardware switch input
  if(Have_switch) {
    debug_log_ln("Switch: init");
    pinMode(SWINPUT, INPUT);
  }

  // Webserver defaults, before config load
  static char webserver_login_str[MAX_HTTPD_LOGIN_STR];
  static char webserver_pass_str[MAX_HTTPD_LOGIN_STR];
  snprintf(webserver_login_str,  MAX_HTTPD_LOGIN_STR, "%s",   HTTPD_LOGIN);
  #ifdef HTTPD_PASS_USE_CHIP_ID
    snprintf(webserver_pass_str, MAX_HTTPD_LOGIN_STR, "%06X", Chip_id);
  #else
    snprintf(webserver_pass_str, MAX_HTTPD_LOGIN_STR, "%s",   HTTPD_PASS);
  #endif
  Webserver_login = webserver_login_str;
  Webserver_pass  = webserver_pass_str;

  // NTP default server, before config load
  static char ntp_server_name[MAX_NTP_SERVER_STR];
  snprintf(ntp_server_name,   MAX_NTP_SERVER_STR, "%s", NTP_SERVER);
  Ntp_server = ntp_server_name;

  // Init Extreme night mode hour bitmask, in case not saved in config yet...
  update_night_extreme_mask();

  // Load persistent configuration
  debug_log("Defaults:   ");
  debug_print_config();
  if( load_config() ) {
    debug_log("Configured: ");
    debug_print_config();
  }
  
  // Handle WiFi setup
  matrix_icon(1);
  display_show_str(1, "NEt");
  altdisplay_show_str("init");

  // WiFi.hostname() can easily cause exception 28.
  if (WiFi.hostname(Device_name)) {
    debug_log_ln("WiFi init: hostname set as '%s'", Device_name);
  } else {
    if (WiFi.hostname(DEVICE_NAME)) {
      debug_log_ln("WiFi init: hostname set as '%s' ('%s' failed)", DEVICE_NAME, Device_name);
    } else {
      debug_log_ln("WiFi init: failed to set hostname. Probably 'ESP-%06X'.", Chip_id);
    }
  }

  WiFiManager wifiManager;
  wifiManager.setAPCallback(wifi_ap_callback);
  wifiManager.autoConnect(Cfg_ap_ssid, Cfg_ap_pass);
  display_ip_addr(1);

  // NTP client startup
  static NTPClient NTPClientObj(Ntp_udp, Ntp_server, Timezone * 3600, Ntp_update_freq * 1000);
  TimeClient = &NTPClientObj;
  TimeClient->on_before_update(on_before_ntp);
  TimeClient->on_after_update(on_after_ntp);
  TimeClient->setEUDST(Ntp_apply_eu_dst);
  if (Clock_drift_comp != 0.0) TimeClient->setDriftComp(Clock_drift_comp);
  TimeClient->begin();

  // Webserver hooks, startup
  setup_httpd();

  // Over-The-Air updates
  setup_ota();

  // Setup complete, LED off
  intled(OFF);

  // Show init sync
  display_point(1, POINT_OFF);
  display_point(2, POINT_OFF);
  display_show_str(1, "SYNC", 300);
  display_show_str(2, "ntP");

  check_luminosity();
  if (Brightness != DEFAULT_BRIGHTNESS)
    set_brightness(Brightness);
}

//
// Set up Over-The-Air updates
// https://randomnerdtutorials.com/esp8266-ota-updates-with-arduino-ide-over-the-air/
//
void setup_ota(void) {
  ArduinoOTA.setHostname(Device_name);
  ArduinoOTA.setRebootOnSuccess(false);
  ArduinoOTA.onStart([]() {
    debug_log_ln("OTA Start");
    if (Night_mode) set_night_mode(0);
    LittleFS.end();
    display_point(1, POINT_OFF);
    display_point(2, POINT_OFF);
    display_show_str(1, "OtA");
    display_show_str(2, "LOAd");
    matrix_icon(2);
  });
  ArduinoOTA.onEnd([]() {
    debug_log_ln("OTA End, rebooting");
    matrix_icon(3);
    display_show_str(1, "init");
    altdisplay_show_str("boot", DISPLAY_DELAY_1);
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pct = (progress / (total / 100));
    char dbuf[8];
    if (pct < 100)
      snprintf(dbuf, sizeof(dbuf), "dL%2u", pct);
    else
      snprintf(dbuf, sizeof(dbuf), "donE");
    display_show_str(1, dbuf);
    debug_log_ln("OTA Download: %03u%%", pct);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    char dbuf[8];
    snprintf(dbuf, sizeof(dbuf), "Err%d", error);
    debug_log("Error - ");
    if (error == OTA_AUTH_ERROR) {
      debug_log_ln("Auth Failed");
      display_show_str(1, "oAut");
    } else if (error == OTA_BEGIN_ERROR) {
      debug_log_ln("Begin Failed");
      display_show_str(1, "obEg");
    } else if (error == OTA_CONNECT_ERROR) {
      debug_log_ln("Connect Failed");
      display_show_str(1, "oCon");
    } else if (error == OTA_RECEIVE_ERROR) {
      debug_log_ln("Receive Failed");
      display_show_str(1, "odnL");
    } else if (error == OTA_END_ERROR) {
      debug_log_ln("End Failed");
      display_show_str(1, "oEnd");
    }
    matrix_char(error);
    display_show_str(2, dbuf);
    delay(DISPLAY_DELAY_3);
    LittleFS.begin();
  });
  ArduinoOTA.begin();
}

//
// Called when net init can not connect to a WiFi network and a WiFi connection
// configuration HTTP server is started
//
void wifi_ap_callback(WiFiManager *WiFiAP) {
  (void)WiFiAP; // unused
  char *ap_ssid_end = &Cfg_ap_ssid[4];
  char *ap_pass_end = &Cfg_ap_pass[4];

  debug_log_ln("NOTICE: Join WiFi network SSID '%s', password '%s',\r\nThen go to http://%s/ to configure network settings.",
    Cfg_ap_ssid, Cfg_ap_pass, WiFi.softAPIP().toString().c_str());

  if (Night_mode) set_night_mode(0);

  display_show_str(1, "conF");
  display_show_str(2, "nEt", 1000);

  display_show_str(1, "Join");
  altdisplay_show_str("SSId", 1000, 1500);
  
  display_show_str(1, Cfg_ap_ssid);
  altdisplay_show_str(ap_ssid_end, 2500, 3000);
  
  display_clear(2);
  display_show_str(1, "PASS", 1000);
  display_show_str(1, Cfg_ap_pass);
  altdisplay_show_str(ap_pass_end, 2500, 3000);
  
  display_show_str(1, "Goto");
  altdisplay_show_str("HttP", 1000, 1000);
  display_ip_addr(WiFi.softAPIP(), 5);
  if (Have_displays) delay(2000); // add more delay

  display_clear(2);
  display_show_str(1, "PASS", DISPLAY_DELAY_2);
  display_show_str(1, Cfg_ap_pass);
  altdisplay_show_str(ap_pass_end, 1000);
}

//
// Called just before the date/time is updated via NTP
//
void on_before_ntp() {
  unsigned long c;
  c = TimeClient->getUpdateCount();
  if (c) { // not initial request
    debug_log_ln("\r\n%s: Starting NTP update %lu from %s",
              TimeClient->getDateTimeISO(), TimeClient->getUpdateCount() + 1, Ntp_server);
    display_point(Dp[2], POINT_OFF);
    display_show_str(Dp[2], "SYNC");
  } else {
    debug_log_ln("Initial NTP request from %s starting", Ntp_server);
  }
}

//
// Called just after the date/time is updated via NTP
//
void on_after_ntp() {
  if (TimeClient->getUpdateCount() == 1)
    Start_time = TimeClient->getEpochRaw(); // start counting uptime
  Clock_drift_comp = TimeClient->getDriftComp();
  debug_log_ln("%s: Time updated", TimeClient->getDateTimeISO());
  Refresh = true;
}

//
// Load configuration
//
bool load_config(void) { return load_config(MAIN_CONFIG_FILE); }
bool load_config(const char *cfg_path) {
  if(LittleFS.exists(cfg_path)) {
    debug_log_ln("Loading configuration from '%s'", cfg_path);
    File cfg = LittleFS.open(cfg_path, "r");
    if (!cfg) {
      debug_log_ln("WARNING: Failed to open the config file '%s'", cfg_path);
      return false;
    }
    char buf[128], *p, *stk;
    while (cfg.available()) {
      int i = cfg.readBytesUntil('\n', buf, sizeof(buf) - 1);
      if (i < 3) continue; // key:value can not be less than 3 chars
      buf[i] = 0; // readBytesUntil doesn't terminate strings
      p = strtok_r(buf,  ":", &stk); 
      p = strtok_r(NULL, ":", &stk);
      if (p == NULL) continue; // no value
      if (Debug_config) debug_log_ln("'%s':'%s'", buf, p);

      if        (strstr(buf, "Name")) {    // Friendly device name (char[])
        snprintf(Device_name, MAX_DEVICE_NAME_STR, "%s", p);
      } else if (strstr(buf, "Debug")) {   // Serial debug output (int, 0 = off, 1 = on)
        p[0] == '1' ? Debug->enable() : Debug->disable();
      } else if (strstr(buf, "TZ")) {      // Timezone (float)
        Timezone = atof(p);
      } else if (strstr(buf, "EUDST")) {   // EU DST conformance (int, 0 = no, 1 = yes)
        Ntp_apply_eu_dst = atoi(p);
      } else if (strstr(buf, "NTPs")) {    // NTP server (char[])
        snprintf(Ntp_server, MAX_NTP_SERVER_STR, "%s", p);
      } else if (strstr(buf, "Poll")) {    // NTP poll frequency (s) with legacy config fallback if < MIN_POLL/1000 (was hours, float)
        Ntp_update_freq = strtoul(p, NULL, 10);
        if (Ntp_update_freq < MIN_POLL / 1000) Ntp_update_freq = (unsigned long)((atof(p) * 3600));
      } else if (strstr(buf, "Drift")) {   // Drift compensation value (double, ms/h)
        Clock_drift_comp = atof(p);
      } else if (strstr(buf, "Pass")) {    // Web server password
        snprintf(Webserver_pass, 32, "%s", p);
      } else if (strstr(buf, "Lang")) {    // Matrix language
        Mtx_wd_lc = atoi(p);
      } else if (strstr(buf, "Hzero")) {   // Hour leading zero, 0 or 1
        H_lead_zero = atoi(p);
      } else if (strstr(buf, "Nite")) {    // Night mode, 0|1|2
        Night_mode = atoi(p);
      } else if (strstr(buf, "NiEx")) {    // Extreme night mode, 0|1
        Night_extreme = atoi(p);
      } else if (strstr(buf, "NiXs")) {    // Extreme night mode start
        NiEx_from = atoi(p); update_night_extreme_mask();
      } else if (strstr(buf, "NiXe")) {    // Extreme night mode end
        NiEx_to = atoi(p);   update_night_extreme_mask();
      } else if (strstr(buf, "NiSw")) {    // Night swap, 0|1
        Night_swap = atoi(p);
      } else if (strstr(buf, "NiLx")) {    // Night mode trigger lux level
        Night_lux = atof(p);
      } else if (strstr(buf, "NMEA")) {    // NMEA mode after silence
        NMEA_init_mode = atoi(p);
      } else if (strstr(buf, "Vunit")) {   // SOG velocity unit
        NMEA_sog_unit = atoi(p);
      } else if (strstr(buf, "Bri")) {     // Brightness level (initial, if Have_gy49)
        Brightness = atoi(p);
      } else if (strstr(buf, "MBr")) {     // Forced Brightness level
        Manual_bright = atoi(p);
      } else if (strstr(buf, "MtxB")) {    // Matrix brightness step table
        parse_mtx_bri_str(p);
      } else if (strstr(buf, "Dp2B")) {    // 2. display brightness step table
        parse_bri_str(2, p);
      } else if (strstr(buf, "Dp1B")) {    // 1. display brightness step table
        parse_bri_str(2, p);
      } else if (strstr(buf, "NiBr")) {    // Night brightness table
        parse_night_bri_str(p);
      } else if (strstr(buf, "LuxSt")) {   // Brightness level steps in lux
        parse_lux_config_str(p, true);
      } else {
        debug_log_ln("Unknown config key '%s'", buf);
      }
    }
    cfg.close();   
    debug_log_ln("Configuration loaded.");
  } else {
    debug_log_ln("NOTICE: Configuration file '%s' missing.", cfg_path);
    return false;
  }
  return true;
}

// Return true if the configured password is the hardcoded default
bool have_default_password(void) {
  return strcmp(Webserver_pass, HTTPD_PASS) ? false : true;
}
// Return true if the password is NOT default OR auth required anyway
bool need_to_authorize(void) {
  if (have_default_password() && NO_AUTH_IF_DEFAULT_PASS) 
    return false;
  return true;
}

//
// Read a string of comma separated values, 
// configure brightness level 1-7 lux values accordingly
//
void parse_lux_config_str(char *str) {
  parse_lux_config_str(str, false);
}
void parse_lux_config_str(char *str, bool reading_config) {
  char *lxp, *lstk;
  int lxi = 0;
  lxp = strtok_r(str, ",", &lstk);
  while (lxp != NULL && lxi <= 7) {
    double lxv = atof(lxp);
    if (lxv < -1.0f) lxv = -1.0f; else if (lxv > 10000.0f) lxv = 10000.0f;
    if (Debug_config) debug_log_ln("  Lx[%d] =%8.2f", lxi, lxv);
    Lux_step[lxi] = lxv;
    lxp = strtok_r(NULL, ",", &lstk);
    lxi++;
  }
  if (reading_config && lxi < 8) {
    // Old 7-level (no level for display off) config
    for (int i = 7; i >= 0; i--) {
      Lux_step[i] = i > 0 ? Lux_step[i-1] : 0.0f;
    }
    if (Debug_config) {
      debug_log_ln("Detected old 7-step Lx config; Shifted:");
      for (int i = 0; i <= 7; i++) {
        debug_log_ln("  Lx[%d] =%8.2f", i, Lux_step[i]);
      }
    }
  }
}

// Display brightness lookup table
void parse_mtx_bri_str(char *str) { parse_bri_str(0, str); }
void parse_night_bri_str(char *str) { parse_bri_str(3, str); }
void parse_bri_str(uint8_t dnum, char *str) {
  const static char *ds[] = { "Mx", "D1", "D2", "Nm" };
  char *lxp, *lstk;
  int lxi = 0;
  lxp = strtok_r(str, ",", &lstk);
  while (lxp != NULL && lxi <= (dnum < 3 ? 8 : 3)) {
    int lxv = atoi(lxp);
    if (lxv < 0) lxv = 0; else if (lxv > 8) lxv = 8;
    if (Debug_config) debug_log_ln("  %s[%d] = %d", ds[dnum], lxi, lxv);
    if (dnum < 3)
      Dp_bright[dnum][lxi] = lxv;
    else 
      Night_bright[lxi] = lxv;
    lxp = strtok_r(NULL, ",", &lstk);
    lxi++;
  }
}

// NiEx_from + NiEx_to --> NiEx_h bitmask
void update_night_extreme_mask(void) {
  NiEx_h = 0;
  if (NiEx_from > 23) NiEx_from = 23;
  if (NiEx_to   > 23) NiEx_to   = 23;
  if (NiEx_from < NiEx_to) {
    for (int i = NiEx_from; i < NiEx_to; i++) {
      NiEx_h |= (1UL << i);
    }
  } else if (NiEx_from > NiEx_to) {
    for (int i = NiEx_from; i < 24; i++) {
      NiEx_h |= (1UL << i);
    }
    for (int i = 0; i < NiEx_to; i++) {
      NiEx_h |= (1UL << i);
    }
  } else {
    NiEx_h = 0xffffff;
  }
}

//
// Save the configuration
//
bool save_config(void) { return save_config(MAIN_CONFIG_FILE); }
bool save_config(const char *cfg_path) {
  char buf[384];
  debug_log_ln("%s: Saving configuration to '%s'", TimeClient->getDateTimeISO(), cfg_path);
  File cfg = LittleFS.open(cfg_path, "w");
  if (!cfg) {
    debug_log_ln("WARNING: Failed to open the config file '%s'", cfg_path);
    return false;
  }
  snprintf(buf, sizeof(buf), "Name:%s\nDebug:%s\nPass:%s\nTZ:%.2f\nEUDST:%d\n"
    "NTPs:%s\nPoll:%luf\nDrift:%.8f\n"
    "Lang:%d\nHzero:%d\nNite:%d\nNiEx:%d\nNiXs:%d\nNiXe:%d\nNiSw:%d\n"
    "NiLx:%.2f\nNiBr:%d,%d,%d\n"
    "NMEA:%d\nVunit:%d\nBri:%d\nMBr:%d\n"
    "Dp1B:%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
    "Dp2B:%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
    "MtxB:%d,%d,%d,%d,%d,%d,%d,%d,%d\n"
    "LuxSt:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    Device_name, Debug->is_enabled() ? "1" : "0", Webserver_pass, Timezone, Ntp_apply_eu_dst, 
    Ntp_server, Ntp_update_freq, Clock_drift_comp,
    Mtx_wd_lc, H_lead_zero, Night_mode, Night_extreme, NiEx_from, NiEx_to, Night_swap, 
    Night_lux, Night_bright[0], Night_bright[1], Night_bright[2], 
    NMEA_init_mode, NMEA_sog_unit, Brightness, Manual_bright,
    Dp1_bright[0], Dp1_bright[1], Dp1_bright[2], Dp1_bright[3], Dp1_bright[4], Dp1_bright[5], Dp1_bright[6], Dp1_bright[7], Dp1_bright[8],
    Dp2_bright[0], Dp2_bright[1], Dp2_bright[2], Dp2_bright[3], Dp2_bright[4], Dp2_bright[5], Dp2_bright[6], Dp2_bright[7], Dp2_bright[8],
    Mtx_bright[0], Mtx_bright[1], Mtx_bright[2], Mtx_bright[3], Mtx_bright[4], Mtx_bright[5], Mtx_bright[6], Mtx_bright[7], Mtx_bright[8],
    Lux_step[0], Lux_step[1], Lux_step[2], Lux_step[3], Lux_step[4], Lux_step[5], Lux_step[6], Lux_step[7]
  );
  if (!cfg.print(buf)) {
    debug_log_ln("WARNING: Failed to write to the config file '%s'", cfg_path);
    return false;
  }
  cfg.close();
  debug_log_ln("%s: Configuration saved.", TimeClient->getDateTimeISO());
  return true;
}

//
//  Print out the configuration (including web config password in plaintext)
//
void debug_print_config(void) {
  debug_log_ln(
    "Name %s, TZ %+.2f, EuDST %s, NTPserver '%s', "
    "PollFreq %lu (%.2f h), DriftC %+.4f ms/h, Bri %d, MBr %d, "
    "Dp1B[%d,%d,%d,%d,%d,%d,%d,%d,%d], "
    "Dp2B[%d,%d,%d,%d,%d,%d,%d,%d,%d], "
    "MtxB[%d,%d,%d,%d,%d,%d,%d,%d,%d], "
    "LuxSt[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], "
    "Pass '%s', HLzero %sd, Nite %s, NiteSwap %sd, ExNite %s (%02d-%02d), NiteLux %.2f, "
    "NiteB[%d,%d,%d], "
    "Mtx %s, NMEA %s, V-unit %s, Debug %s",
    Device_name, 
    Timezone, 
    (Ntp_apply_eu_dst ? "ON" : "OFF"), 
    Ntp_server, 
    Ntp_update_freq, (float)Ntp_update_freq / 3600.0,
    Clock_drift_comp, 
    Brightness, 
    Manual_bright,
    Dp1_bright[0], Dp1_bright[1], Dp1_bright[2], Dp1_bright[3], Dp1_bright[4], Dp1_bright[5], Dp1_bright[6], Dp1_bright[7], Dp1_bright[8],
    Dp2_bright[0], Dp2_bright[1], Dp2_bright[2], Dp2_bright[3], Dp2_bright[4], Dp2_bright[5], Dp2_bright[6], Dp2_bright[7], Dp2_bright[8],
    Mtx_bright[0], Mtx_bright[1], Mtx_bright[2], Mtx_bright[3], Mtx_bright[4], Mtx_bright[5], Mtx_bright[6], Mtx_bright[7], Mtx_bright[8],
    Lux_step[0], Lux_step[1], Lux_step[2], Lux_step[3], Lux_step[4], Lux_step[5], Lux_step[6], Lux_step[7], 
    Webserver_pass, 
    Hlzero_str[H_lead_zero],
    Night_mode_str[Night_mode],
    Night_swap_str[Night_swap],
    Night_extreme_str[Night_extreme],
    NiEx_from, NiEx_to,
    Night_lux, 
    Night_bright[0], Night_bright[1], Night_bright[2], 
    Have_matrix ? Mtx_wd_lang[Mtx_wd_lc] : "N/A", 
    Have_nmea ? NMEA_mode_str[NMEA_init_mode] : "N/A",
    Have_nmea ? Sog_unit_str[NMEA_sog_unit] : "N/A",
    Debug->is_enabled() ? "ON" : "OFF"
  );
}

//
// get hardware configuration and some key options
//
const char *get_hwcfg_str(void) {
  return get_hwcfg_str(0);
}
const char *get_hwcfg_str(int html) {
  static char buf[MAX_HTTPD_OUT] = "";
  
  static const char *dp_type[] = { "N/A", "Clock", "Decimal" };
  static const char *endis[]   = { "Disabled", "Enabled" };
  static const char *avail[]   = { "N/A", "Configured" };

  const char *sr, *sp, *er;
  if (html) { sr = sr_a[1]; sp = sp_a[1]; er = er_a[1]; } 
  else      { sr = sr_a[0]; sp = sp_a[0]; er = er_a[0]; }

  snprintf(buf, sizeof(buf), 
    "%sDevice type%s%s%s"
    "%sHostname%s%s%s"
    "%sChip id%sESP_%06X%s"
    "%sMac address%s%s%s"
    "%sSoftware version%s%s%s"
    "%sSoftware build%s%s%s"
    "%sToolkits%s%s%s"
    "%sPrimary display%s%s%s"
    "%sSecondary display%s%s%s"
    "%sMatrix display%s%s%s"
    "%sInternal LED%s%s%s"
    "%sLight sensor%s%s%s"
    "%sMode button%s%s%s"
    "%sNMEA modes%s%s%s"
    "%sDebug to serial%s%s%s",
    sr, sp, DEVICE_NAME, er,
    sr, sp, Device_name, er,
    sr, sp, Chip_id, er,
    sr, sp, WiFi.macAddress().c_str(), er,
    sr, sp, MASTER_VERSION, er,
    sr, sp, Build, er,
    sr, sp, ESP.getFullVersion().c_str(), er,
    sr, sp, Have_dp[1] ? (Decimal_dp[1] ? dp_type[2] : dp_type[1]) : dp_type[0], er,
    sr, sp, Have_dp[2] ? (Decimal_dp[1] ? dp_type[2] : dp_type[1]) : dp_type[0], er,
    sr, sp, Have_matrix         ? avail[1] : avail[0], er,
    sr, sp, Have_intled         ? endis[1] : endis[0], er,
    sr, sp, Have_gy49           ? avail[1] : avail[0], er,
    sr, sp, Have_switch         ? avail[1] : avail[0], er,
    sr, sp, Have_nmea           ? endis[1] : endis[0], er,
    sr, sp, Debug->is_enabled() ? endis[1] : endis[0], er
  );
  return buf;
}

//
// get current timezone/settings as string
//
const char *get_timezone_str(void) {
  return get_timezone_str(TimeClient->getEpochTime(), 0); }
const char *get_timezone_str(int html) {
  return get_timezone_str(TimeClient->getEpochTime(), html); }
const char *get_timezone_str(double epochtime) {
  return get_timezone_str(epochtime, 0); }
const char *get_timezone_str(double epochtime, int html) {
  static char buf[128] = "";
  int dst = 0;

  const char *sp; 
  sp = html ? sp_a[1] : sp_a[0];

  if (Ntp_apply_eu_dst) {
    dst = TimeClient->isDST(epochtime);
    snprintf(buf, sizeof(buf), 
      "Timezone%s%+.2f h, current offset %+.2f h (EU DST followed, %s time)",
      sp, Timezone, Timezone + dst, dst ? "summer" : "normal");
  } else {
    snprintf(buf, sizeof(buf), 
      "Timezone%s%+.2f h (static, DST rules not followed)", sp, Timezone);
  }
  return buf;
}

//
// format seconds as "0d 00:00:00" 
//
const char *get_uptime_str(void) {
  return get_uptime_str((unsigned long)(TimeClient->getEpochTime() - Start_time));
}
const char *get_uptime_str(unsigned long seconds) {
  static char buf[256] = "";
  unsigned int days = seconds / 86400UL; seconds %= 86400UL;
  unsigned int hours = seconds / 3600;   seconds %= 3600;
  unsigned int minutes = seconds / 60;   seconds %= 60;
  snprintf(buf, sizeof(buf), "%u day%s, %02u:%02u:%02lu", 
    days, days == 1 ? "" : "s", hours, minutes, seconds);
  return buf;
}

//
// Get luminance
//
const char *get_lux_str(void) {
  return get_lux_str(0); }
const char *get_lux_str(int html) {
  int pbri, sbri, mbri;
  const static char *niexs[] = { "normal", "extreme" };
  static char buf[512] = "";
  static char mtxbuf[128] = "";
  static char dp1buf[128] = "";
  static char dp2buf[128] = "";
  static char nite[80] = "";

  const char *sr, *sp, *er;
  if (html) { sr = sr_a[1]; sp = sp_a[1]; er = er_a[1]; } 
  else      { sr = sr_a[0]; sp = sp_a[0]; er = er_a[0]; }

  if ((Night_mode == NIGHT_MODE_ON) || (Night_mode && Luminance <= Night_lux)) {
    pbri = Night_bright[1];
    sbri = Night_bright[2];
    mbri = Night_bright[0];
    snprintf(nite, sizeof(nite), "%s/Active (%s)", 
      Night_mode_str[Night_mode],
      niexs[Night_extreme]
    );
  } else {
    if (Manual_bright < 0) {
      pbri = Dp1_bright[Brightness];
      sbri = Dp2_bright[Brightness];
      mbri = Mtx_bright[Brightness];
    } else {
      pbri = Dp1_bright[Manual_bright];
      sbri = Dp2_bright[Manual_bright];
      mbri = Mtx_bright[Manual_bright];
    }
    snprintf(nite, sizeof(nite), "%s/Inactive (%s)", 
      Night_mode_str[Night_mode],
      niexs[Night_extreme]
    );
  }

  if (Have_dp[1]) {
    snprintf(dp1buf, sizeof(dp1buf), 
      "%sPrimary display%slevel %d, levels %d,%d,%d,%d,%d,%d,%d,%d,%d%s",
      sr, sp, pbri, Dp1_bright[0], Dp1_bright[1], Dp1_bright[2], Dp1_bright[3], Dp1_bright[4], Dp1_bright[5], Dp1_bright[6], Dp1_bright[7], Dp1_bright[8], er
    );
  }
  if (Have_dp[2]) {
    snprintf(dp2buf, sizeof(dp2buf), 
      "%sSecondary display%slevel %d, levels %d,%d,%d,%d,%d,%d,%d,%d,%d%s",
      sr, sp, sbri, Dp2_bright[0], Dp2_bright[1], Dp2_bright[2], Dp2_bright[3], Dp2_bright[4], Dp2_bright[5], Dp2_bright[6], Dp2_bright[7], Dp2_bright[8], er
    );
  }
  if (Have_matrix) {
    snprintf(mtxbuf, sizeof(mtxbuf), 
      "%sMatrix%slevel %d, levels %d,%d,%d,%d,%d,%d,%d,%d,%d, %s%s",
      sr, sp, mbri, Mtx_bright[0], Mtx_bright[1], Mtx_bright[2], Mtx_bright[3], Mtx_bright[4], Mtx_bright[5], Mtx_bright[6], Mtx_bright[7], Mtx_bright[8], Mtx_wd_lang[Mtx_wd_lc], er
    );
  }
  if (Have_gy49) {
    snprintf(buf, sizeof(buf), 
      "%sAmbient light%s%.2f lux, level %d (%s)%s"
      "%sBrightness steps%s%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f lux%s"
      "%s%s%s"
      "%sHour leading zero%s%sd%s"
      "%sNight mode%s%s%s",
      sr, sp, Luminance, Brightness, (Manual_bright < 0) ? "auto" : "manual", er,
      sr, sp, Lux_step[0], Lux_step[1], Lux_step[2], Lux_step[3], Lux_step[4], Lux_step[5], Lux_step[6], Lux_step[7], er,
      dp1buf, 
      dp2buf, 
      mtxbuf, 
      sr, sp, Hlzero_str[H_lead_zero], er,
      sr, sp, nite, er
    );
  } else {
    snprintf(buf, sizeof(buf), 
      "%sNo light sensor, set level%s%d%s"
      "%sHour leading zero%s%sd%s"
      "%sNight mode%s%s%s"
      "%s%s%s", 
      sr, sp, Brightness, er,
      sr, sp, Hlzero_str[H_lead_zero], er,
      sr, sp, nite, er,
      dp1buf, 
      dp2buf, 
      mtxbuf
    );
  }
  return buf;
}

//
// Get drift table
//
const char *get_drift_str(void) {
  int l = 0;
  static char buf[(DRIFT_ARRAY_SIZE + 1) * 25 + 1];
  if(TimeClient->getDriftSampleCount()) {
    l = TimeClient->getDriftValues(buf, "%+7.1f %+7.1f %+7.1f\r\n", sizeof(buf));
    if (l < 0) debug_log_ln("WARNING: get_drift_str buffer (%u) too small: %d", sizeof(buf), l);
  } else {
    snprintf(buf, sizeof(buf), "No clock drift values available yet.\r\n");
  }
  return buf;
}

//
// Get system info output
//
const char *get_info_str(void) { return get_info_str(0); }
const char *get_info_str(int html) {
  static char buf[MAX_HTTPD_OUT];
  char current_ts[MAX_ISO_TS];
  char update_ts[MAX_ISO_TS];
  char upd_err[32+MAX_ISO_TS];

  const char *uptime_p;
  const char *tzstr_p;

  double        current_epoch =  TimeClient->getEpochTime();
  double        update_epoch =   TimeClient->getLastUpdateEpoch();
  double        err_epoch =      TimeClient->getLastErrEpoch();
  
  unsigned long ntp_update_ct =  TimeClient->getUpdateCount();
  unsigned long ntp_error_ct =   TimeClient->getErrorCount();
  unsigned long since_update_s = (millis() - TimeClient->getLastUpdateMillis()) / 1000;

  static char nmea_details[128] = "";

  const char *sr, *sp, *er;
  if (html) { sr = sr_a[1]; sp = sp_a[1]; er = er_a[1]; } 
  else      { sr = sr_a[0]; sp = sp_a[0]; er = er_a[0]; }

  TimeClient->getDateTimeISO(current_ts, current_epoch);
  TimeClient->getDateTimeISO(update_ts,  update_epoch);

  uptime_p = get_uptime_str((unsigned long)(current_epoch - Start_time));
  tzstr_p  = get_timezone_str(current_epoch, html);

  if (ntp_error_ct) {
    snprintf(upd_err, sizeof(upd_err), 
      ", %lu error%s%s",
      ntp_error_ct,
      (ntp_error_ct == 1) ? " at " : "s, last ",
      TimeClient->getLastErrMillis()
        ? TimeClient->getDateTimeISO(err_epoch) : "at init");
  } else {
    snprintf(upd_err, sizeof(upd_err), ", no errors");
  }

  #ifdef HAVE_NMEA
  snprintf(nmea_details, sizeof(nmea_details), 
    "%sNMEA mode%s%s (default %s), V-unit %s%s",
    sr, sp, NMEA_mode_str[NMEA_mode],
    NMEA_mode_str[NMEA_init_mode],
    Sog_unit_str[NMEA_sog_unit], er
  );
  #endif

  snprintf(buf, sizeof(buf), 
    "%sUptime%s%s%s"
    "%s"
    "%s%s%s" // tzstr
    "%sNTP server%s%s%s"
    "%sNTP update count%s%lu%s%s"
    "%sLast update%s%s, %lu s ago%s"
    "%sNext update%s%s%s"
    "%sCurrent time%s%s%s"
    "%sDrift compensation%s%+.4f ms/h%s",
    sr, sp, uptime_p, er,
    nmea_details,
    sr, tzstr_p, er,
    sr, sp, Ntp_server, er,
    sr, sp, ntp_update_ct, upd_err, er,
    sr, sp, update_ts, since_update_s, er,
    sr, sp, TimeClient->getNextUpdateTime(true), er,
    sr, sp, current_ts, er,
    sr, sp, Clock_drift_comp, er
  ); 

  return buf;
}

// HTML header string, optional additional part of <title>
void send_html_header(void) { send_html_header(NULL); return; }
void send_html_header(const char *title) {
  static char buf[MAX_HTTPD_OUT];
  snprintf(buf, sizeof(buf), 
    "<!DOCTYPE html>\r\n"
    "<html>\r\n"
    "<head>\r\n"
    "  <meta charset='UTF-8'>\r\n"
    "  <link rel=icon href='data:,'>\r\n"
    "  <title>%s%s%s</title>\r\n"
    "  <style>\r\n",
    Device_name, title ? " - " : "", title ? title : ""
  );
  Httpd.sendContent(buf);

  send_html_style();
      
  snprintf(buf, sizeof(buf), 
    "</style>\r\n"
    "</head>\r\n"
    "<body>\r\n"
    "<h1><a href='/'>%s</a>%s%s</h1>\r\n"
    "<h2>%s</h2>\r\n",
    Device_name, title ? " - " : "", title ? title : "",
    TimeClient->getDateTimeISO()
  );
  Httpd.sendContent(buf);

  return;
}

void send_html_style(void) { 
  Httpd.sendContent(F(
    "@import url('https://fonts.googleapis.com/css2?family=PT+Sans&family=Roboto:wght%40300&family=Roboto+Mono:wght%40300&display=swap');\r\n"
  ));
  Httpd.sendContent(F(
    "body{font-family:'PT Sans',sans-serif;margin:0;padding:1em 2em;color:#d7cfbd;background-color:#333;}"
    "p{font-size:110%}"
    "ul{padding:0 1.25rem;list-style-type:disclosure-closed;}"
    "li{line-height:1.5;}"
    "a{font-size:120%;color:#ffecdd;text-decoration:none;padding-right:.33rem;}"
    "h1 a{font-size:100%;}"
    "h4{font-size:120%;}"
    "pre{margin:0;padding:0;line-height:1.1;}"
    "input{padding:3px .4em;color:#fff;background-color:#222;border:none;font-size:110%;}"
    "input[type=submit],input[type=reset]{padding:4px .6em;margin:1em .7em 1.5em 0;cursor:pointer;color:#d4c7ad;background-color:#555;border:1px solid #000;font-weight:bold;}"
    "input[type=radio]{display:none;}"
    "input[type=radio]:checked+label{color:#fff;background-color:#111;}"
    "input[type=text]{margin-right:.3em;}"
    "label{display:inline-block;cursor:pointer;padding:2px .8em;background-color:#555;border:1px solid #000;}"
    "table{border:none;border-collapse:collapse;}"
    "td,pre{color:#c8c1b2;}"
    "td,th{text-align:left;padding:1px .5em 0;}"
    "th{font-weight:normal;}"
    "h2,h4,td,label,input,.set,#ft{font-family:Roboto,sans-serif;font-weight:lighter;}"
    "p,h1,h2,h3,h4{margin:.8rem 0 .33rem;}"
    "#ft{font-size:70%;padding:2rem .33rem;color:#847868;}"
    ".set{margin:0 .8em;color:#b9c4d5;}"
    ".note{padding-left:22em;color:#888;font-size:80%;}"
    ".ms{font-family:'Roboto mono',monospace;}\r\n"
  ));
}

void send_html_footer(void) { 
  Httpd.sendContent(F("<p id=ft>Janne Korkkula 2021</p></body></html>\r\n"));
  return;
}

bool html_start(void) { return html_start(NULL); }
bool html_start(const char *title) {
  debug_log_ln("%s: httpd - request%s%s", TimeClient->getDateTimeISO(), title ? ": " : "", title ? title : "");
  if (!Httpd.chunkedResponseModeStart(200, MIME_HTML)) {
    Httpd.send(505, MIME_HTML, F("HTTP/1.1 required"));
    return false;
  }
  send_html_header();
  return true;
}

void html_end(void) { html_end(true); }
void html_end(bool show_footer) {
  if (show_footer) send_html_footer();
  Httpd.chunkedResponseFinalize();
}

//
// set up the web server
//
void setup_httpd(void) {
  Httpd.on("/",             httpd_root);
  Httpd.on("/mode",         httpd_mode);
  Httpd.on("/swap",         httpd_swap_displays);
  #ifdef HAVE_NMEA
  Httpd.on("/unit",         httpd_sog_unit);
  #endif
  Httpd.on("/config",       httpd_config);
  Httpd.on("/opts",         httpd_options);
  Httpd.on("/passw",        httpd_cfgpassw);
  Httpd.on("/time",         httpd_gettime);
  Httpd.on("/info",         httpd_getinfo);
  Httpd.on("/lux",          httpd_getlux);
  Httpd.on("/sync",         httpd_ntpsync);
  Httpd.on("/save",         httpd_saveconfig);
  Httpd.on("/reboot",       httpd_reboot);
  Httpd.on("/set",          httpd_setconfig);
  Httpd.on("/pwset", HTTP_POST, httpd_setpassw);
  Httpd.begin();
  debug_log_ln("Web server listening at http://%s:%d/", WiFi.localIP().toString().c_str(), HTTPD_PORT);
}

bool httpd_check_auth(void) {
  if (need_to_authorize()) {
    if(!Httpd.authenticate(Webserver_login, Webserver_pass)) {
      Httpd.requestAuthentication();
      return false;
    }
  }
  return true;
}

void httpd_root(void) {
  char out[MAX_HTTPD_OUT];
  char lob[MAX_HTTPD_SS] = "";

  if (!html_start()) return;

  #ifdef HAVE_NMEA
  if (NMEA_cog_d > -1.0 && NMEA_sog_d > -1.0) {
    snprintf(out, sizeof(out), 
      "<h4>NMEA %sactive, last COG %.1f&deg;T, SOG %.1f %s</h4>\r\n",
      NMEA_mode ? "" : "in", NMEA_cog_d, NMEA_sog_d * Sog_multiplier[NMEA_sog_unit], Sog_unit_str[NMEA_sog_unit]
    );
  } else {
    snprintf(out, sizeof(out), 
      "<h4>NMEA inactive, no data received</h4>\r\n"
    );
  }
  Httpd.sendContent(out);
  #endif

  if (Have_gy49) {
    snprintf(out, sizeof(out), 
      "<h4>%.2f lux, brightness level %d</h4>\r\n", Luminance, Brightness);
    Httpd.sendContent(out);
  }

  snprintf(out, sizeof(out), 
    "<ul><li><a href='/info'>System info</a></li>\r\n"
    "</ul><ul>"
    "<li><a href='/config'>Main configuration</a> *</li>\r\n");
  Httpd.sendContent(out);

  if (Have_displays) Httpd.sendContent(F("<li><a href='/opts'>Display options</a> *</li>\r\n"));

  if (need_to_authorize()) snprintf(lob, sizeof(lob), " (login '%s')", Webserver_login);
  snprintf(out, sizeof(out), 
    "<li><a href='/passw'>%s password</a>%s *</li>\r\n"
    "</ul><ul>",
        have_default_password() ? "Set" : "Change", lob);
  Httpd.sendContent(out);

  if (Have_displays) {
    snprintf(out, sizeof(out), 
      "<li><a href='/mode'>Switch display mode</a> ' <span class=set>%s</span></li>\r\n",     
      Quiet_mode ? "normal" : "seconds");
    Httpd.sendContent(out);
  }

  if (Have_displays > 1) {
    Httpd.sendContent(F("<li><a href='/swap'>Swap displays</a> (primary/secondary) '</li>\r\n"));
  }

  #ifdef HAVE_NMEA
  snprintf(out, sizeof(out), 
    "<li><a href='/unit'>Cycle SOG unit</a> (kn, km/h, m/s) <span class=set>%s</span></li>\r\n",
    Sog_unit_str[NMEA_sog_unit]
  );
  Httpd.sendContent(out);
  #endif

  Httpd.sendContent(F(
    "<li><a href='/time'>Current time</a> (plaintext)</li>\r\n"
    "</ul><ul>"
    "<li><a href='/sync'>Force NTP sync</a> (Varying intervals may reduce accuracy) *</li>\r\n"
    "<li><a href='/save'>Save config/drift</a> (done automatically each midnight) *</li>\r\n"
    "<li><a href='/reboot'>Reboot</a> (undo unsaved changes) *</li>\r\n"
  ));

  snprintf(out, sizeof(out), 
    "</ul><p class=note>* %s<br>' not saved in config</p>",
    need_to_authorize() ? "requires auth" : "requires auth if pass set"
  );
  Httpd.sendContent(out);

  html_end();
}

void httpd_mode(void) {
  debug_log_ln("%s: httpd - request: mode", TimeClient->getDateTimeISO());
  toggle_mode();
  Httpd.sendHeader("Location", String("/"), true);
  Httpd.send(302, MIME_TEXT, "");
}

void httpd_swap_displays(void) {
  debug_log_ln("%s: httpd - request: swap", TimeClient->getDateTimeISO());
  display_swap();
  Refresh = true;
  Httpd.sendHeader("Location", String("/"), true);
  Httpd.send(302, MIME_TEXT, "");
}

#ifdef HAVE_NMEA
void httpd_sog_unit(void) {
  NMEA_sog_unit++;
  if (NMEA_sog_unit > 2) NMEA_sog_unit = 0;
  debug_log_ln("%s: httpd - request: unit: set to %s", TimeClient->getDateTimeISO(), Sog_unit_str[NMEA_sog_unit]);
  Refresh = true;
  Httpd.sendHeader("Location", String("/"), true);
  Httpd.send(302, MIME_TEXT, "");
}
#endif

void httpd_gettime(void) {
  char ts[MAX_ISO_TS + 2];
  snprintf(ts,  sizeof(ts),  "%s", TimeClient->getDateTimeISO());
  debug_log_ln("%s: httpd - request: time", ts);
  strncat(ts, "\r\n", (sizeof(ts) - strlen(ts) - 1));
  Httpd.send(200, MIME_TEXT, ts);
}

void httpd_getlux(void) {
  debug_log_ln("%s: httpd - request: lux", TimeClient->getDateTimeISO());
  Httpd.send(200, MIME_TEXT, get_lux_str());
}

void httpd_getinfo(void) {
  char out[MAX_HTTPD_OUT];

  if (!html_start("info")) return;

  if (TimeClient->getDriftSampleCount() && TimeClient->getUpdateCount() > 1) {
    Httpd.sendContent(F("<h3>Clock drift stats (newest first)</h3>\r\n"));
    snprintf(out, sizeof(out), "<pre class=ms>%7s %7s %7s\r\n%7s %7s %7s\r\n%s</pre>\r\n", 
      "drift", "comp", "err", "ms/h", "ms/h", "ms", get_drift_str());
    Httpd.sendContent(out);
  } else {
    Httpd.sendContent(F("<h3>Clock drift stats not available yet</h3>\r\n"));
  }

  Httpd.sendContent(F("<h3>Runtime information</h3>\r\n"));
  snprintf(out, sizeof(out), "<table>%s</table>\r\n", get_info_str(1));
  Httpd.sendContent(out);

  /*
  Httpd.sendContent(F("<h3>Displays, brightness</h3>\r\n"));
  snprintf(out, sizeof(out), "<table>%s</table>\r\n", get_lux_str(1));
  Httpd.sendContent(out);
  */

  Httpd.sendContent(F("<h3>Hardware configuration</h3>\r\n"));
  snprintf(out, sizeof(out), "<table>%s</table>\r\n", get_hwcfg_str(1));
  Httpd.sendContent(out);

  html_end();
}

void httpd_reboot(void) {
  if (!httpd_check_auth()) return;
  if (!html_start("reboot")) return;
  Httpd.sendContent(F("<h4>Restarting...</h4>\r\n"));
  html_end();
  delay(1000);
  ESP.restart();
}

void httpd_ntpsync(void) {
  char ts[MAX_ISO_TS];
  char out[1024];

  if (!httpd_check_auth()) return;
  if (!html_start("sync")) return;
  
  TimeClient->getDateTimeISO(ts);
  
  snprintf(out, sizeof(out), "<pre>Starting NTP sync at %s\r\n", ts);
  Httpd.sendContent(out);

  if (TimeClient->forceUpdate()) {
    snprintf(out, sizeof(out),    "Time updated         %s\r\n</pre>\r\n", TimeClient->getDateTimeISO());
  } else {
    snprintf(out, sizeof(out),    "Update failed        %s\r\n</pre>\r\n", TimeClient->getDateTimeISO());
  }
  Httpd.sendContent(out);

  html_end();
}

void httpd_config(void) {
  char out[MAX_HTTPD_OUT];

  if (!httpd_check_auth()) return;
  if (!html_start("config")) return;

  Httpd.sendContent(F(
    "<form action='/set' method=post autocomplete=off>\r\n"
  ));

  snprintf(out, sizeof(out), 
    "<p>Hostname: <input type=text name=Name placeholder='%s'></p>\r\n"
    "<p>NTP server name/ip: <input type=text name=NTPs placeholder='%s'></p>\r\n"
    "<p>NTP poll frequency, h: <input type=text name=Poll size=6 pattern='[0-9.]+[0-9]*' placeholder='%.2f'></p>\r\n"
    "<p>Drift compensation, ms: <input type=text name=Drift size=10 pattern='[-+0-9.]+[0-9]*' placeholder='%+.4f'></p>\r\n",
    Device_name,
    Ntp_server, 
    (float)Ntp_update_freq / 3600,
    Clock_drift_comp
  );
  Httpd.sendContent(out);

  snprintf(out, sizeof(out), 
    "<p>UTC offset, h: <input type=text name=TZ size=6 maxlength=6 pattern='[-+0-9.]+[0-9]*' placeholder='%+.2f'></p>\r\n"
    "<p>EU daylight saving:<span class=set>%sd</span>"
      "<input type=radio name=EUDST id=euA value=1><label for=euA>Enable</label> "
      "<input type=radio name=EUDST id=euB value=0><label for=euB>Disable</label></p>\r\n"
    "<p>Debug to serial:<span class=set>%sd</span>"
      "<input type=radio name=Debug id=dbA value=1><label for=dbA>Enable</label> "
      "<input type=radio name=Debug id=dbB value=0><label for=dbB>Disable</label></p>\r\n",
    Timezone, Ntp_apply_eu_dst ? "Enable" : "Disable",
    Debug->is_enabled() ? "Enable" : "Disable"
  );
  Httpd.sendContent(out);

  Httpd.sendContent(F(
    "<input type=submit value=Configure> <input type=reset value=Reset>\r\n"
    "</form>"
  ));

  html_end();
}

void httpd_options(void) {
  char out[MAX_HTTPD_OUT];

  if (!httpd_check_auth()) return;
  if (!html_start("options")) return;

  Httpd.sendContent(F(
    "<form action='/set' method=post autocomplete=off>\r\n"
  ));

  snprintf(out, sizeof(out), 
    "<p>Hour leading zero:<span class=set>%sd</span>"
    "<input type=radio name=Hzero id=hzA value=1>"
    "<label for=hzA>%s</label> "
    "<input type=radio name=Hzero id=hzB value=0>"
    "<label for=hzB>%s</label></p>\r\n",
    Hlzero_str[H_lead_zero], Hlzero_str[1], Hlzero_str[0]
  );
  Httpd.sendContent(out);

  snprintf(out, sizeof(out), 
    "<p>Night mode:<span class=set>%s</span>"
    "<input type=radio name=Nite id=nmA value=%d>"
    "<label style=display:%s for=nmA>%s</label> "
    "<input type=radio name=Nite id=nmB value=%d>"
    "<label for=nmB>%s</label> "
    "<input type=radio name=Nite id=nmC value=%d>"
    "<label for=nmC>%s</label></p>\r\n",
    Night_mode_str[Night_mode], 
    NIGHT_MODE_AUTO, Have_gy49 ? "inline-block" : "none", Night_mode_str[NIGHT_MODE_AUTO],
    NIGHT_MODE_OFF, Night_mode_str[NIGHT_MODE_OFF], 
    NIGHT_MODE_ON, Night_mode_str[NIGHT_MODE_ON]
  );
  Httpd.sendContent(out);

  snprintf(out, sizeof(out), 
    "<p>Extreme night mode:<span class=set>%sd</span>"
    "<input type=radio name=NiEx id=neA value=1>"
    "<label for=neA>%s</label> "
    "<input type=radio name=NiEx id=neB value=0>"
    "<label for=neB>%s</label> &nbsp; "
    "starts (hour) <input type=text size=4 placeholder='%d' pattern='[0-9]+' name=NiXs>"
    "ends at <input type=text size=4 placeholder='%d' pattern='[0-9]+' name=NiXe>"
    "</p>\r\n",
    Night_extreme_str[Night_extreme], Night_extreme_str[1], Night_extreme_str[0],
    NiEx_from, NiEx_to
  );
  Httpd.sendContent(out);

  if (Have_displays > 1) {
    snprintf(out, sizeof(out), 
      "<p>Swap displays in night mode:<span class=set>%sd</span>"
      "<input type=radio name=NiSw id=nsA value=1>"
      "<label for=nsA>%s</label> "
      "<input type=radio name=NiSw id=nsB value=0>"
      "<label for=nsB>%s</label></p>\r\n",
      Night_swap_str[Night_swap], Night_swap_str[1], Night_swap_str[0]
    );
    Httpd.sendContent(out);
  }

  #ifdef HAVE_NMEA
  snprintf(out, sizeof(out), 
    "<p>NMEA mode (remembered):<span class=set>%s</span>"
    "<input type=radio name=NMEA id=eA value=1>"
    "<label for=eA>%s</label> "
    "<input type=radio name=NMEA id=eB value=2>"
    "<label for=eB>%s</label> "
    "<input type=radio name=NMEA id=eC value=3>"
    "<label for=eC>%s</label> "
    "<input type=radio name=NMEA id=eD value=4>"
    "<label for=eD>%s</label> "
    "<input type=radio name=NMEA id=eE value=5>"
    "<label for=eE>%s</label></p>\r\n",
     NMEA_mode_str[NMEA_init_mode], 
     NMEA_mode_str[1], NMEA_mode_str[2], NMEA_mode_str[3], NMEA_mode_str[4], NMEA_mode_str[5]
  );
  Httpd.sendContent(out);
  #endif

  if(Have_matrix) {
    snprintf(out, sizeof(out), 
      "<p>Matrix language:<span class=set>%s</span>"
      "<input type=radio name=Lang id=mlA value=0>"
      "<label for=mlA>%s</label> "
      "<input type=radio name=Lang id=mlB value=1>"
      "<label for=mlB>%s</label></p>\r\n",
      Mtx_wd_lang[Mtx_wd_lc], Mtx_wd_lang[0], Mtx_wd_lang[1]
    );
    Httpd.sendContent(out);
  }

  if(Have_gy49) {
    snprintf(out, sizeof(out), 
      "<p>Brightness (0-8, -1 = auto): <input type=text name=MBr size=3 maxlength=3 pattern='-?[0-8]' placeholder='%d'></p>\r\n"
      "<p>Brightness levels at lux (now %.2f lux, level %d):<br>\r\n",
      Manual_bright, Luminance, Brightness
    );
    Httpd.sendContent(out);
    for (int i = 0; i <= 7; i++) {
      snprintf(out, sizeof(out), 
        "%d:<input type=text name=Lx%d size=6 placeholder='%.2f' pattern='[0-9.]+'>%s",
        i+1, i, Lux_step[i], i < 7 ? " " : "</p>\r\n"
      );
      Httpd.sendContent(out);
    }
    snprintf(out, sizeof(out), 
      "<p>Night mode trigger level, lux: <input type=text name=NiLx size=8 maxlength=8 pattern='[0-9.]+' placeholder='%.2f'></p>\r\n",
      Night_lux
    );
    Httpd.sendContent(out);
  } else if (Have_displays) {
    snprintf(out, sizeof(out),
      "<p>Brightness, 0-8: <input type=text name=Bri size=3 pattern='[0-8]' placeholder='%d'></p>\r\n",
      Brightness
    );
    Httpd.sendContent(out);
  }

  if (Have_displays || Have_matrix) {
    Httpd.sendContent("<p>Night mode levels: &nbsp; ");
    if (Have_dp[1]) {
      snprintf(out, sizeof(out), 
        "Primary: <input type=text name=Nt1 size=4 placeholder='%d' pattern='[0-9]'>%s",
        Night_bright[1], (Have_dp[2] || Have_matrix) ? " " : "</p>\r\n"
      );
      Httpd.sendContent(out);
    }
    if (Have_dp[2]) {
      snprintf(out, sizeof(out), 
        "Secondary: <input type=text name=Nt2 size=4 placeholder='%d' pattern='[0-9]'>%s",
        Night_bright[2], Have_matrix ? " " : "</p>\r\n"
      );
      Httpd.sendContent(out);
    }
    if (Have_matrix) {
      snprintf(out, sizeof(out), 
        "Matrix: <input type=text name=Nt0 size=4 placeholder='%d' pattern='[0-9]'></p>\r\n",
        Night_bright[0]
      );
      Httpd.sendContent(out);
    }
  }

  if(Have_dp[1]) {
    snprintf(out, sizeof(out), 
      "<p>Primary display brightness for levels 0-8 (now at level %d)<br>\r\n", Dp1_bright[Brightness]
    );
    Httpd.sendContent(out);
    for (int i = 0; i <= 8; i++) {
      snprintf(out, sizeof(out), 
        "%d:<input type=text name=Pl%d size=4 placeholder='%d' pattern='[0-9]'>%s",
        i, i, Dp1_bright[i], i < 8 ? " " : "</p>\r\n"
      );
      Httpd.sendContent(out);
    }
  }

  if(Have_dp[2]) {
    snprintf(out, sizeof(out), 
      "<p>Second display brightness for levels 0-8 (now at level %d)<br>\r\n", Dp2_bright[Brightness]
    );
    Httpd.sendContent(out);
    for (int i = 0; i <= 8; i++) {
      snprintf(out, sizeof(out), 
        "%d:<input type=text name=Ds%d size=4 placeholder='%d' pattern='[0-9]'>%s",
        i, i, Dp2_bright[i], i < 8 ? " " : "</p>\r\n"
      );
      Httpd.sendContent(out);
    }
  }

  if(Have_matrix) {
    snprintf(out, sizeof(out), 
      "<p>Matrix brightness for levels 0-8 (now at level %d)<br>\r\n", Mtx_bright[Brightness]
    );
    Httpd.sendContent(out);
    for (int i = 0; i <= 8; i++) {
      snprintf(out, sizeof(out), 
        "%d:<input type=text name=Mx%d size=4 placeholder='%d' pattern='[0-9]'>%s",
        i, i, Mtx_bright[i], i < 8 ? " " : "</p>\r\n"
      );
      Httpd.sendContent(out);
    }
  }

  Httpd.sendContent(F(
    "<input type=submit value=Configure> <input type=reset value=Reset>\r\n"
    "</form>"
  ));

  html_end();
}

void httpd_cfgpassw(void) {
  char out[MAX_HTTPD_OUT];
  
  snprintf(out, sizeof(out), "%s password", (need_to_authorize() ? "change" : "set"));
  if (!httpd_check_auth()) return;
  if (!html_start(out)) return;

  Httpd.sendContent(F("<form action='/pwset' method=post autocomplete=off>\r\n"));

  if(strcmp(Webserver_pass, HTTPD_PASS) || !NO_AUTH_IF_DEFAULT_PASS) {
    snprintf(out, sizeof(out), 
      "<p>Default password '%s'%s</p>\r\n"
      "<p>OLD password: <input type=password maxlength=%d name=P0></p>\r\n",
      HTTPD_PASS, (NO_AUTH_IF_DEFAULT_PASS ? " (set as this to disable auth)" : ""),
      MAX_HTTPD_LOGIN_STR
    );
    Httpd.sendContent(out);
  } 

  snprintf(out, sizeof(out), 
    "<p>New password: <input type=password maxlength=%d name=P1><br>\r\n"
    "New password: <input type=password maxlength=%d name=P2 placeholder='Please repeat'></p>\r\n"
    "<br><input type=submit value='Change password'>\r\n"
    "</form>",
    MAX_HTTPD_LOGIN_STR, MAX_HTTPD_LOGIN_STR
  );
  Httpd.sendContent(out);

  html_end();
}

void httpd_setpassw(void) {
  char p1[MAX_HTTPD_LOGIN_STR] = "";
  bool ok[3] = { false, false, false };
  static const char errmsg[] = "Invalid password change request\r\n";

  if (!httpd_check_auth()) return;
  if (!html_start("update password")) return;

  for (int i = 0; i < Httpd.args(); i++ ) {
    char buf[128], p[128];
    Httpd.argName(i).toCharArray(buf, 128);
    Httpd.arg(i).toCharArray(p, 128);
    if (!p[0]) {
      continue;
    } else if (strstr(buf, "P0")) {
      if (!strcmp(p, Webserver_pass)) ok[0] = true; // same
    } else if (strstr(buf, "P1")) {
      snprintf(p1, sizeof(p1), "%s", p);
    } else if (strstr(buf, "P2")) {
      if (!strcmp(p, p1)) ok[1] = true; // verify matches
    }
  }
  if (have_default_password())    ok[0] = true; // default password, allow change
  if (strcmp(Webserver_pass, p1)) ok[2] = true; // not same as before

  if (ok[0] && ok[1] && ok[2]) {
    debug_log_ln("Setting password to '%s'", p1);
    snprintf(Webserver_pass, MAX_HTTPD_LOGIN_STR, "%s", p1);
    Httpd.sendContent(F("<h4>Password updated</h4><form action='/save'><input type='submit' value='Save config (new password) to flash'></form>"));
  } else {
    debug_log(errmsg);
    Httpd.sendContent(errmsg);
  }

  html_end();
}

void httpd_saveconfig(void) {
  if (!httpd_check_auth()) return;
  if (!html_start("save config")) return;

  debug_print_config();
  if (save_config()) {
    Httpd.sendContent(F("<h4>Configuration saved.</h4>\r\n"));
  } else { 
    Httpd.sendContent(F("<p>ERROR: Failed to save the configuration!</p>\r\n"));
  }

  html_end();
}

void httpd_setconfig(void) {
  char ss[MAX_HTTPD_SS];
  static const char *dn[] = { "matrix", "primary", "secondary" };
  int ok = 0;
  int sbri = 0;
  static const char errmsg[] = "Invalid config request\r\n";

  if (!httpd_check_auth()) return;
  if (!html_start("configuring")) return;

  Httpd.sendContent("<p>");

  for (int i = 0; i < Httpd.args(); i++ ) {
    char buf[128], p[128];
    Httpd.argName(i).toCharArray(buf, 128);
    Httpd.arg(i).toCharArray(p, 128);

    if (!p[0]) continue;
    
    // Friendly device name (char[])
    else if (strstr(buf, "Name")) {
      snprintf(ss, sizeof(ss), "Setting device name = '%s'", p);
      _httpd_setconfig_ss(ss, &ok); // debug print and output, increment ok
      snprintf(Device_name, MAX_DEVICE_NAME_STR, "%s", p);
    } 

    // Timezone (float)
    else if (strstr(buf, "TZ")) {
      float f = atof(p);
      snprintf(ss, sizeof(ss), "Setting TZ = %+.2f", f);
      _httpd_setconfig_ss(ss, &ok);
      Timezone = f;
      TimeClient->setTimeOffset(Timezone * 3600L);
      Refresh = true;
    } 

    // EU DST conformance (int, 0 = no, 1 = yes)
    else if (strstr(buf, "EUDST")) {
      int d = atoi(p);
      snprintf(ss, sizeof(ss), "Setting EU DST %s", d ? "ON" : "OFF");
      _httpd_setconfig_ss(ss, &ok);
      Ntp_apply_eu_dst = d;
      TimeClient->setEUDST(Ntp_apply_eu_dst);
      Refresh = true;
    } 

    // NTP server (char[])
    else if (strstr(buf, "NTPs")) {
      snprintf(ss, sizeof(ss), "Setting NTP server '%s'", p);
      _httpd_setconfig_ss(ss, &ok);
      snprintf(Ntp_server, MAX_NTP_SERVER_STR, "%s", p);
      TimeClient->setNtpServer(Ntp_server);
    } 

    // NTP poll frequency (float, hours)
    else if (strstr(buf, "Poll")) {
      double f = atof(p);
      unsigned long v = (unsigned long)(f * 3600);
      if (v >= MIN_POLL / 1000) {
        snprintf(ss, sizeof(ss), "Setting NTP poll frequency %.2f h (%lu s)", f, v);
      } else {
        v = MIN_POLL / 1000;
        snprintf(ss, sizeof(ss), "Setting minimum NTP poll frequency: %lu s", v);
      }
      _httpd_setconfig_ss(ss, &ok);
      Ntp_update_freq = v;
      TimeClient->setUpdateInterval(Ntp_update_freq * 1000);
    } 

    // Drift compensation value (double, ms/h)
    else if (strstr(buf, "Drift")) {
      double f = atof(p);
      if (f == 0.0) snprintf(ss, sizeof(ss), "Clearing drift compensation data");
      else snprintf(ss, sizeof(ss), "Setting drift compensation %+.4f ms/h", f);
      _httpd_setconfig_ss(ss, &ok);
      Clock_drift_comp = f;
      TimeClient->setDriftComp(Clock_drift_comp);
    } 

    // Matrix weekday language (int)
    else if (strstr(buf, "Lang")) {
      int d = atoi(p);
      if (d > 1) d = 1; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting matrix language: %s", Mtx_wd_lang[d]);
      _httpd_setconfig_ss(ss, &ok);
      Mtx_wd_lc = d;
      Refresh = true;
    } 

    // Hour leading zero (int)
    else if (strstr(buf, "Hzero")) {
      int d = atoi(p);
      if (d > 1) d = 1; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting hour leading zero: %s", d ? "ON" : "OFF");
      _httpd_setconfig_ss(ss, &ok);
      H_lead_zero = d;
      Refresh = true;
    } 

    // Night mode (int)
    else if (strstr(buf, "Nite")) {
      int d = atoi(p);
      if (d > 2) d = 2; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting night mode: %s", Night_mode_str[d]);
      _httpd_setconfig_ss(ss, &ok);
      set_night_mode(d);
    } 

    // Extreme night mode (int)
    else if (strstr(buf, "NiEx")) {
      int d = atoi(p);
      if (d > 1) d = 1; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting extreme night mode: %s", Night_mode_str[d]);
      _httpd_setconfig_ss(ss, &ok);
      Night_extreme = d;
      set_night_mode(Night_mode);
    } 

    // Extreme night mode start
    else if (strstr(buf, "NiXs")) {
      int d = atoi(p);
      if (d > 23) d = 23; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting extreme night mode start: %02d:00", d);
      _httpd_setconfig_ss(ss, &ok);
      NiEx_from = d;
      update_night_extreme_mask();
      set_night_mode(Night_mode);
    } 

    // Extreme night mode end
    else if (strstr(buf, "NiXe")) {
      int d = atoi(p);
      if (d > 23) d = 23; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting extreme night mode end: %02d:00", d);
      _httpd_setconfig_ss(ss, &ok);
      NiEx_to = d;
      update_night_extreme_mask();
      set_night_mode(Night_mode);
    } 

    // Night swap (int)
    else if (strstr(buf, "NiSw")) {
      int d = atoi(p);
      if (d > 1) d = 1; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting night swap: %s", Night_swap_str[d]);
      _httpd_setconfig_ss(ss, &ok);
      set_night_swap(d);
    } 

    // Night mode trigger level, lux
    else if (strstr(buf, "NiLx")) {
      float f = atof(p);
      snprintf(ss, sizeof(ss), "Setting night mode trigger level to  %.2f lux", f);
      _httpd_setconfig_ss(ss, &ok);
      Night_lux = f;
      sbri++;
    } 

    // NMEA init mode (int)
    else if (strstr(buf, "NMEA")) {
      int d = atoi(p);
      if (d > 5) d = 5; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting NMEA init mode: %s", NMEA_mode_str[d]);
      _httpd_setconfig_ss(ss, &ok);
      if (NMEA_mode) NMEA_mode = d;
      NMEA_init_mode = d;
      Refresh = true;
    } 

    // (Default) brightness level (int)
    else if (strstr(buf, "Bri")) {
      int d = atoi(p);
      if (d > 8) d = 8; else if (d < 0) d = 0;
      snprintf(ss, sizeof(ss), "Setting default brightness level %d", d);
      _httpd_setconfig_ss(ss, &ok);
      Brightness = d;
      sbri++;
    } 

    // Forced brightness level (int)
    else if (strstr(buf, "MBr")) {
      int d = atoi(p);
      if (d > 8) d = 8; else if (d < -1) d = -1;
      if (d > -1)
        snprintf(ss, sizeof(ss), "Setting manual brightness level %d", d);
      else 
        snprintf(ss, sizeof(ss), "Setting auto brightness");
      _httpd_setconfig_ss(ss, &ok);
      Manual_bright = d;
      sbri++;
    } 

    // Brightness level steps in lux (externally parsed)
    else if (strstr(buf, "LuxSt")) {
      snprintf(ss, sizeof(ss), "Setting lux levels '%s'", p);
      _httpd_setconfig_ss(ss, &ok);
      parse_lux_config_str(p);
      sbri++;
    } 

    // Primary brightness table (externally parsed)
    else if (strstr(buf, "Dp1St")) {
      snprintf(ss, sizeof(ss), "Setting primary display levels '%s'", p);
      _httpd_setconfig_ss(ss, &ok);
      parse_bri_str(1, p);
      sbri++;
    } 

    // Secondary brightness table (externally parsed)
    else if (strstr(buf, "Dp2St")) {
      snprintf(ss, sizeof(ss), "Setting secondary display levels '%s'", p);
      _httpd_setconfig_ss(ss, &ok);
      parse_bri_str(2, p);
      sbri++;
    } 

    // Matrix brightness table (externally parsed)
    else if (strstr(buf, "MtxSt")) {
      snprintf(ss, sizeof(ss), "Setting matrix levels '%s'", p);
      _httpd_setconfig_ss(ss, &ok);
      parse_mtx_bri_str(p);
      sbri++;
    } 

    // Night mode brightness table (externally parsed)
    else if (strstr(buf, "NiBr")) {
      snprintf(ss, sizeof(ss), "Setting night levels '%s'", p);
      _httpd_setconfig_ss(ss, &ok);
      parse_night_bri_str(p);
      sbri++;
    } 

    // Single brightness level step (positive float)
    else if (strstr(buf, "Lx")) {
      int d = atoi(buf + 2);
      double f = atof(p);
      if (d >= 0 && d <= 7) {
        snprintf(ss, sizeof(ss), "Setting brightness step %d to %.3f lx", d, f);
        _httpd_setconfig_ss(ss, &ok);
        Lux_step[d] = f;
      }
      sbri++;
    } 

    // Single primary display level
    else if (strstr(buf, "Pl")) {
      int d = atoi(buf + 2);
      int l = atoi(p);
      if (d >= 0 && d <= 8 && l >= 0) {
        snprintf(ss, sizeof(ss), "Setting primary display level %d to %d", d, l);
        _httpd_setconfig_ss(ss, &ok);
        Dp1_bright[d] = l;
      }
      sbri++;
    } 

    // Single secondary display level
    else if (strstr(buf, "Ds")) {
      int d = atoi(buf + 2);
      int l = atoi(p);
      if (d >= 0 && d <= 8 && l >= 0) {
        snprintf(ss, sizeof(ss), "Setting secondary display level %d to %d", d, l);
        _httpd_setconfig_ss(ss, &ok);
        Dp2_bright[d] = l;
      }
      sbri++;
    } 

    // Single matrix level
    else if (strstr(buf, "Mx")) {
      int d = atoi(buf + 2);
      int l = atoi(p);
      if (d >= 0 && d <= 8 && l >= 0) {
        snprintf(ss, sizeof(ss), "Setting matrix level %d to %d", d, l);
        _httpd_setconfig_ss(ss, &ok);
        Mtx_bright[d] = l;
      }
      sbri++;
    } 

    // Single night level
    else if (strstr(buf, "Nt")) {
      int d = atoi(buf + 2);
      int l = atoi(p);
      if (d >= 0 && d <= 2 && l >= 0) {
        snprintf(ss, sizeof(ss), "Setting %s night level to %d", dn[d], l);
        _httpd_setconfig_ss(ss, &ok);
        Night_bright[d] = l;
      }
      sbri++;
    } 

    // Serial debug output
    else if (strstr(buf, "Debug")) {
      int d = atoi(p);
      if (d) Debug->enable(); // always output a state change message
      snprintf(ss, sizeof(ss), "Setting debug output %s", d ? "ON" : "OFF");
      _httpd_setconfig_ss(ss, &ok);
      if (!d) Debug->disable(); // kill output afterwards
    } 

    // debug NTP drift
    else if (strstr(buf, "debugdrift")) {
      int d = atoi(p);
      snprintf(ss, sizeof(ss), "Setting drift debugging %s", d ? "ON" : "OFF");
      _httpd_setconfig_ss(ss, &ok);
      TimeClient->setDebugDrift(d ? true : false);
    } 

    // debug NTP packets
    else if (strstr(buf, "debugpkt")) {
      int d = atoi(p);
      snprintf(ss, sizeof(ss), "Setting NTP packet debugging %s", d ? "ON" : "OFF");
      _httpd_setconfig_ss(ss, &ok);
      TimeClient->setDebugPackets(d ? true : false);
    } 
    
    else if (strstr(buf, "plain")) {
      // 2.5 introduced an artifact...
      ;
    }
    else {
      snprintf(ss, sizeof(ss), "Unknown config key '%s'", buf);
      _httpd_setconfig_ss(ss, NULL);
    }
  }

  if (ok) {
    // Something was configured
    Httpd.sendContent(F(
      "</p><form action='/save'><input type='submit' value='Save config to flash'></form>"
    ));
  } else {
    debug_log(errmsg);
    Httpd.sendContent(errmsg);
    Httpd.sendContent("</p>");
  }

  html_end();
  if (sbri) set_brightness();
}
// debug print and output
void _httpd_setconfig_ss(char *ss, int *ok) {
  debug_log_ln(ss);
  Httpd.sendContent(ss);
  Httpd.sendContent("<br>\r\n");
  if (ok) (*ok)++;
}


//
// Check for requests sent from the serial port
//
bool check_serial_input(void) {
  bool ds = Debug->is_enabled();
  size_t rb   = 0; // available/read bytes
  static size_t bufs = MAX_SERIAL_INPUT - 1; // buffer space left
  static char   buf[MAX_SERIAL_INPUT];       // command buffer
  char          rx[MAX_SERIAL_INPUT];        // serial read buffer
  char         *eol; // position of EOL in rx, NULL while incomplete
  unsigned long ul; // temporary storage

  rb = Serial.available();
  if (!rb) return false;

  rb = Serial.readBytes(rx, min((sizeof(rx) - 1), rb));
  rx[rb] = 0; // readBytes does NOT terminate.

  eol =           strchr(rx, '\r'); // got CR?
  if (!eol) eol = strchr(rx, '\n'); // got LF?
  if (!eol) eol = strchr(rx, 3);    // got Control-C?

  if (!buf[0] && rx != eol) Serial.print(">> "); // local echo, start
  if (!eol) {
    Serial.print(rx);                            // local echo, append
  } else {
    if (*eol != 3) *eol = 0;                     // strip EOL
    if (buf[0]) Serial.println(rx);              // local echo, end
  }

  if (bufs >= rb) {
    strncat(buf, rx, bufs); 
    bufs -= rb;
  } else { // buffer full, silently clear & ignore
    buf[0] = 0; bufs = MAX_SERIAL_INPUT - 1;
    Serial.println("...");
  }

  if (!eol) return false;
  if (*eol) { // Control-C
    buf[0] = 0; bufs = MAX_SERIAL_INPUT - 1;
    return false;
  }

  // OK, parse/process command
  if (!ds) Debug->enable(); // would be stupid not to output anything.

  #ifdef HAVE_NMEA
  if (buf[0] == '$') {
    //debug_log_ln("$ from input");
    if (strstr(buf+3, "RMC,") && strstr(buf+12, ",A,")) {
      // valid GNSS RMC sentence detected
      parse_nmea_rmc(buf);
    }
  } else 
  #endif
  if (strstr(buf, "info")) {
    debug_log(get_hwcfg_str(0));
    debug_log(get_info_str(0));
    if (TimeClient->getDriftSampleCount() > 0 && TimeClient->getUpdateCount() > 1) {
      debug_log_ln("Clock drift stats (newest first):");
      debug_log_ln("%7s %7s %7s", "drift", "comp", "err");
      debug_log_ln("%7s %7s %7s", "ms/h", "ms/h", "ms");
      debug_log(get_drift_str());
    } else {
      debug_log_ln("Clock drift stats not available yet");
    }
  } else if (strstr(buf, "mode")) {
    Quiet_mode = !Quiet_mode;
    Refresh = true;
    debug_log_ln(Quiet_mode ? "Normal display mode set" : "Seconds display mode set");
  } else if (strstr(buf, "swap")) {
    display_swap();
    Refresh = true;
  } else if (strstr(buf, "unit")) {
    NMEA_sog_unit++; if (NMEA_sog_unit > 2) NMEA_sog_unit = 0;
    debug_log_ln("SOG unit set to %s", Sog_unit_str[NMEA_sog_unit]);
    Refresh = true;
  } else if (strstr(buf, "time")) {
    debug_log_ln("%s\r\nCurrent time: %s", get_timezone_str(), TimeClient->getDateTimeISO());
  } else if (strstr(buf, "ip")) {
    display_ip_addr();
  } else if (strstr(buf, "lux")) {
    debug_log(get_lux_str(0));
  } else if (strstr(buf, "config")) {
    debug_print_config();
    if (!ds) debug_log_ln("Debug output disabled");
  } else if (strstr(buf, "save")) {
    save_config();
  } else if (strstr(buf, "load")) {
    load_config();
    debug_print_config();
  } else if (strstr(buf, "sync")) {
    TimeClient->forceUpdate();
  } else if (strstr(buf, "nodebug")) {
    debug_log_ln("Debug output disabled");
    Debug->disable();
  } else if (strstr(buf, "debug")) {
    ds = true;
    debug_log_ln("Debug output enabled");
  } else if (strstr(buf, "poll=")) {
    char *p = strchr(buf, '=') + 1;
    ul = strtoul(p, NULL, 10);
    if (ul <= MIN_POLL / 1000) ul = MIN_POLL / 1000;
    debug_log_ln("NTP poll frequency: %lu s", ul);
    Ntp_update_freq = ul;
    TimeClient->setUpdateInterval(Ntp_update_freq * 1000);
  } else if (strstr(buf, "drift=")) {
    char *p = strchr(buf, '=') + 1;
    debug_log_ln("Drift compensation: %.2f ms/h", atof(p));
    Clock_drift_comp = atof(p);
    TimeClient->setDriftComp(Clock_drift_comp);
  } else if (strstr(buf, "tz=")) {
    char *p = strchr(buf, '=') + 1;
    debug_log_ln("Time zone: %+.2f", atof(p));
    Timezone = atof(p);
    TimeClient->setTimeOffset(Timezone * 3600L);
    Refresh = true;
  } else if (strstr(buf, "lang=")) {
    char *p = strchr(buf, '=') + 1;
    uint8_t d = atoi(p);
    if (d < 0) { d = 0; } else if (d > 1) { d = 1; }
    debug_log_ln("Matrix language: '%s'", Mtx_wd_lang[d]);
    Mtx_wd_lc = d;
    Refresh = true;
  } else if (strstr(buf, "hlzero=")) {
    char *p = strchr(buf, '=') + 1;
    uint8_t d = atoi(p);
    if (d < 0) d = 0; else if (d > 1) d = 1;
    debug_log_ln("Hour leading zero %s", d ? "enabled" : "disabled");
    H_lead_zero = d;
    Refresh = true;
  } else if (strstr(buf, "nite=")) {
    char *p = strchr(buf, '=') + 1;
    uint8_t d = atoi(p);
    if (d < 0) d = 0; else if (d > 2) d = 2;
    debug_log_ln("Night mode %s", Night_mode_str[d]);
    set_night_mode(d);
  } else if (strstr(buf, "niex=")) {
    char *p = strchr(buf, '=') + 1;
    uint8_t d = atoi(p);
    if (d < 0) d = 0; else if (d > 1) d = 1;
    debug_log_ln("Extreme night mode %s", Night_extreme_str[d]);
    Night_extreme = d;
    set_night_mode(Night_mode);
  } else if (strstr(buf, "nixs=")) {
    char *p = strchr(buf, '=') + 1;
    uint8_t d = atoi(p);
    if (d < 0) d = 0; else if (d > 23) d = 23;
    debug_log_ln("Extreme night mode start %02d h", d);
    NiEx_from = d;
    update_night_extreme_mask();
    set_night_mode(Night_mode);
  } else if (strstr(buf, "nixe=")) {
    char *p = strchr(buf, '=') + 1;
    uint8_t d = atoi(p);
    if (d < 0) d = 0; else if (d > 23) d = 23;
    debug_log_ln("Extreme night mode end %02d h", d);
    NiEx_to = d;
    update_night_extreme_mask();
    set_night_mode(Night_mode);
  } else if (strstr(buf, "nisw=")) {
    char *p = strchr(buf, '=') + 1;
    uint8_t d = atoi(p);
    if (d < 0) d = 0; else if (d > 1) d = 1;
    debug_log_ln("Night swap %s", Night_swap_str[d]);
    set_night_swap(d);
  } else if (strstr(buf, "nibr=")) {
    char *p = strchr(buf, '=') + 1;
    debug_log_ln("Night mode trigger: %.2f lux", atof(p));
    Night_lux = atof(p);
    if (Night_mode) {
      set_brightness();
      Refresh = true;
    }
  } else if (strstr(buf, "bri=")) {
    char *p = strchr(buf, '=') + 1;
    int d = atoi(p);
    if (d > 8) d = 8; else if (d < -1) d = -1;
    if (d > -1)
      debug_log_ln("Manual brightness %d set (-1 for auto)", d);
    else
      debug_log_ln("Auto brightness set", d);
    Manual_bright = d;
    set_brightness((d > -1) ? d : Brightness);
    Refresh = true;
  } else if (strstr(buf, "lxst=")) {
    char *p = strchr(buf, '=') + 1;
    parse_lux_config_str(p);
    set_brightness();
    Refresh = true;
  } else if (strstr(buf, "dp1b=")) {
    char *p = strchr(buf, '=') + 1;
    parse_bri_str(1, p);
    set_brightness();
    Refresh = true;
  } else if (strstr(buf, "dp2b=")) {
    char *p = strchr(buf, '=') + 1;
    parse_bri_str(2, p);
    set_brightness();
    Refresh = true;
  } else if (strstr(buf, "mtxb=")) {
    char *p = strchr(buf, '=') + 1;
    parse_mtx_bri_str(p);
    set_brightness();
    Refresh = true;
  } else if (strstr(buf, "nibr=")) {
    char *p = strchr(buf, '=') + 1;
    parse_night_bri_str(p);
    if (Night_mode) {
      set_brightness();
      Refresh = true;
    }
  } else if (strstr(buf, "pass=")) {
    char *p = strchr(buf, '=');
    if (p && *++p) {
      snprintf(Webserver_pass, MAX_HTTPD_LOGIN_STR, "%s", p);
      debug_log_ln("Password changed to '%s'", Webserver_pass);
    } else {
      debug_log_ln("Can't set an empty password");
    }
  } else if (strstr(buf, "reboot")) {
    debug_log_ln("Restarting...");
    delay(500);
    ESP.restart();
  } else if (strstr(buf, "hel") || strchr(buf, '?')) { // recognizes 'hello' :)
    debug_log_ln("Serial commands: info, time, ip, config, lux, mode, swap, unit, sync, debug, nodebug, pass=newpw, bri=-1-8, dp2b|mtxb=0,1,,8, lxst=0.0,1,,7, hlzero=0|1, nite=0|1, lang=0|1, tz=[-]h, poll=s, drift=ms, save, load, reboot");
  } else if (buf[0]) {
    debug_log_ln("Unknown serial command, try 'help'");
  }
  if (!ds) Debug->disable(); // disable if was disabled.

  // OK, clear buffer for next commmand
  buf[0] = 0; bufs = MAX_SERIAL_INPUT - 1;
  return true;
}

//
// Ambient light level measurement
//   if (stepped) level has changed,
//   calls set_brightness(level) and returns true, else false
//
bool check_luminosity(void) {
  #ifdef HAVE_GY49
  static int prev_bri = -1; // Comparison value
  static int prev_nite = -1.0f; // Night mode comparison
  int nite = -1; // is-it-night-mode

  Wire.beginTransmission(GY49_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(GY49_ADDR, 2);

  unsigned int luxdata[2];
  if (Wire.available() == 2) {
    luxdata[0] = Wire.read();
    luxdata[1] = Wire.read();
  } else {
    return false; // Should not happen.
  }

  // Convert the data to lux
  int   exponent  =  (luxdata[0] & 0xF0) >> 4;
  int   mantissa  = ((luxdata[0] & 0x0F) << 4) | (luxdata[1] & 0x0F);
  float luminance = pow(2, exponent) * mantissa * 0.045;

  int brightness = 0; // Preset to OFF before level check loop
  for (int i = 7; i >= 0; i--) {
    if (luminance >= Lux_step[i]) {
      brightness = i+1; // 1-8
      break;
    }
  }

  if (Night_mode == NIGHT_MODE_AUTO)
    nite = (luminance <= Night_lux) ? 1 : 0; 

  Luminance = luminance;
  Brightness = brightness;

  if (brightness != prev_bri || nite != prev_nite) {
    if (Debug->is_enabled()) // may happen frequently, avoid unnecessary getDateTimeISO()
      debug_log_ln("%s: Ambient level %d, night mode %sactive (dp1 %d, dp2 %d, mtx %d): %7.2f lux", 
        TimeClient ? TimeClient->getDateTimeISO() : "Init", brightness, 
        (nite > 0 ? "" : "in"),
        Dp1_bright[brightness], Dp2_bright[brightness], Mtx_bright[brightness], luminance);
    prev_bri = brightness; // Store comparison value
    prev_nite = nite;
    if (Night_mode && Night_swap) {
      if (luminance > Night_lux) {
        display_swap(false);
      } else {
        display_swap(true);
      }
    }
    set_brightness(brightness);
    return true; // Intensity changed
  }
  #endif
  return false; // Intensity unchanged or no sensor available
}

//
// Check if WiFi is connected
//
bool check_wifi_status(void) {
  static int prev_st = WiFi.status();
  int st = WiFi.status();
  if (st != prev_st) {
    prev_st = st;
    if (st != WL_CONNECTED) {
      display_point(Altdisplay, POINT_OFF);
      display_show_str(Altdisplay, "noIP", DISPLAY_DELAY_2);
      if (Have_dp[2]) {
        Quiet_mode = true;
      } else {
        Refresh = true;
      }
      return false;
    } else {
      Refresh = true;
    }
  }
  return true;
}

//
// Check if IP address has changed
//
bool check_ip_addr(void) {
  static IPAddress prev_ip = WiFi.localIP();
  static int prev_st = WiFi.status();
  IPAddress ip = WiFi.localIP();
  int st = WiFi.status();
  if (st != prev_st) {
    debug_log_ln("%s: WiFi SSID '%s' status %d: %s%s%s", 
      TimeClient ? TimeClient->getDateTimeISO() : "Init", 
      WiFi.SSID().c_str(), st, 
      st <= MAX_WL_STATUS_ENUM ? WiFi_status[st] : "unknown",
      st == WL_CONNECTED ? ", IP address " : "",
      st == WL_CONNECTED ? WiFi.localIP().toString().c_str() : ""
    );
    prev_st = st;
  }
  if (ip && ip != prev_ip) {
    display_ip_addr(2);
    Refresh = true;
    prev_ip = ip;
    return true;
  }
  return false;
}

//
// Fancy IP address display
//
void display_ip_addr(void)            { display_ip_addr(WiFi.localIP(), 0); }
void display_ip_addr(int header_mode) { display_ip_addr(WiFi.localIP(), header_mode); }
void display_ip_addr(IPAddress ip)    { display_ip_addr(ip, 0); }
void display_ip_addr(IPAddress ip, int header_mode) {
  int st = 0;
  int prev_nite = Night_mode;
  char ts[40];
  if (TimeClient) snprintf(ts, sizeof(ts), "%s: ", TimeClient->getDateTimeISO());

  if (ip == WiFi.localIP()) {
    st = WiFi.status();
    switch (header_mode) {
      case 5: case 4: case 3: break;
      case 2:
        debug_log_ln("%sWiFi SSID '%s', IP address changed to %s", 
                     ts, WiFi.SSID().c_str(), ip.toString().c_str());
        break;
      case 1:
        debug_log_ln("WiFi connected to SSID '%s', IP address %s", 
                     WiFi.SSID().c_str(), ip.toString().c_str());
        break;
      default:
        debug_log_ln("%sWiFi SSID '%s', IP address %s, status %d: %s",
                     ts, WiFi.SSID().c_str(), ip.toString().c_str(),
                     st, st <= MAX_WL_STATUS_ENUM ? WiFi_status[st] : "unknown");
    }
  } else {
    debug_log_ln("%sIP address is %s", ts, ip.toString().c_str());
    if (header_mode == 3) return;
  }

  if (Night_mode) set_night_mode(0);
  matrix_icon(1);
  if (Have_displays) {
    char ipb[5] = "";
    for (uint8_t i = 1; i <= Have_displays; i++) {
      display_point(i, POINT_OFF);
    }
    if (header_mode != 5) {
      display_show_str(1, "IP");
      altdisplay_show_str("Addr", DISPLAY_DELAY_3);
    }
    snprintf(ipb, 5, "%4d", ip[0]);
    display_show_str(1, ipb);
    snprintf(ipb, 5, "%4d", ip[1]);
    altdisplay_show_str(ipb, DISPLAY_DELAY_3);
    snprintf(ipb, 5, "%4d", ip[2]);
    display_show_str(1, ipb);
    snprintf(ipb, 5, "%4d", ip[3]);
    altdisplay_show_str(ipb, DISPLAY_DELAY_3 * 1.5);
  }

  if (prev_nite) set_night_mode(prev_nite);
  Refresh = true;
}

//
// Wrapper functions for controlling displays;
// #ifdef hardware-exists is handled in these,
// so forget about it elsewhere in the main code,
// just go ahead and do anything to non-existing
// displays. The cost is insignificant anyway.
//
void display_clear(uint8_t dnum) {
  // Re-use: TM1637.cpp init() function just clears the display
  display_init(dnum);
}

void display_init(uint8_t dnum) {
  TM1637 *dp = NULL;
  if (dnum == 1) {
    #ifdef HAVE_DP1
    dp = &display1;
    #endif
  } else if (dnum == 2) {
    #ifdef HAVE_DP2
    dp = &display2;
    #endif
  }
  if (dp) dp->init();
}

void display_brightness(uint8_t dnum, int bri) {
  TM1637 *dp = NULL;
  int v = Dp_bright[dnum][bri];
  if (dnum == 1) {
    #ifdef HAVE_DP1
    dp = &display1;
    #endif
  } else if (dnum == 2) {
    #ifdef HAVE_DP2
    dp = &display2;
    #endif
  }
  if (dp) dp->set(v ? v - 1 : -8); // TM1637 kludge: 0-7 on, -8 off
}

void set_night_mode(void) { 
  set_night_mode(Night_mode ? 0 : 1);
};
void set_night_mode(uint8_t mode) {
  Night_mode = mode;
  set_night_swap(Night_swap);
}
void set_night_swap(void) { 
  set_night_swap(Night_swap ? 0 : 1);
};
void set_night_swap(uint8_t mode) {
  Night_swap = mode;
  if (Night_swap && ((Night_mode == 1) || (Night_mode && Brightness <= 1))) {
    display_swap(true);
  } else {
    display_swap(false);
  }
  set_brightness();
}
void cycle_night_mode(void) { cycle_night_mode(1); }
void cycle_night_mode(int dir) {
  const static char *m1 = "nItE";
  const static char *m2[] = { "OFF", "On", "Auto" };
  int nm = Night_mode;

  if (Have_gy49) {
    if (dir > 0) {
      if (nm < 2) nm++;
      else nm = 0;
    } else {
      if (nm > 0) nm--;
      else nm = 2;
    }
  } else {
    // auto mode pointless without light sensor
    nm = nm ? 0 : 1;
  }

  if (dir > 0) {
    for (uint8_t i = 1; i <= Have_displays; i++) {
      display_point(i, POINT_OFF);
    }
    matrix_clear();
    set_night_mode(0);
    display_show_str(1, m1);
    altdisplay_show_str(m2[nm], DISPLAY_DELAY_3 * 1.5);
  }
  if (Debug->is_enabled())
    debug_log_ln("%s: Night mode: %s", TimeClient ? TimeClient->getDateTimeISO() : "Init", Night_mode_str[nm]);

  set_night_mode(nm);
}

void toggle_mode(void) {
  if (NMEA_mode && NMEA_mode < 5) Quiet_mode = true;
  else if (NMEA_mode == 5)        Quiet_mode = false;
  else                            Quiet_mode = !Quiet_mode;

  if ((Night_mode == NIGHT_MODE_ON) || (Night_mode && Brightness <= 1)) {
    // disable night mode if sub-second mode requested while in night mode.
    Prev_nm = Night_mode;
    if (!Quiet_mode) {
      set_night_mode(NIGHT_MODE_OFF);
    }
  } else if (Prev_nm != NIGHT_MODE_OFF) {
    set_night_mode(Prev_nm);
    Prev_nm = NIGHT_MODE_OFF;
  }
  if (Quiet_mode) Show_year = 3; // show current year for 1-2s when going quiet
  Refresh = true;
}

void display_swap(void) {
  display_swap(!Swap_displays);
}
void display_swap(bool mode) {
  if (mode == Swap_displays) return;
  if (Debug->is_enabled())
    debug_log_ln("%s: Swap displays: %s", 
      TimeClient ? TimeClient->getDateTimeISO() : "Init", mode ? "ON" : "OFF");
  if (mode) {
    Dp[1] = 2; Dp[2] = 1;
  } else {
    Dp[1] = 1; Dp[2] = 2;
  }
  Swap_displays = mode;
  Refresh = true;
}

void display_point(uint8_t dnum, uint8_t state) {
  TM1637 *dp = NULL;
  if (dnum == 1) {
    #ifdef HAVE_DP1
    dp = &display1;
    #endif
  } else if (dnum == 2) {
    #ifdef HAVE_DP2
    dp = &display2;
    #endif
  }
  if (dp) dp->point(state);
}

void display_show_str(uint8_t dnum, const char *str) {
  display_show_str(dnum, str, 0);
}
void display_show_str(uint8_t dnum, const char *str, unsigned int delay_after) {
  char buf[5];          // room for string termination,
  strncpy(buf, str, 5); // which strncpy SHOULD automatically zero-pad
  for (uint8_t i = 0; i < 4; i++) {
    unsigned char c = buf[i] ? buf[i] : ' ';
    display_show_char(dnum, i, c);
  }
  if (delay_after) delay(delay_after);
}

void altdisplay_show_str(const char *str) {
  altdisplay_show_str(str, DISPLAY_DELAY_2, 0);
}
void altdisplay_show_str(const char *str, unsigned int delay_after) {
  altdisplay_show_str(str, DISPLAY_DELAY_2, delay_after);
}
void altdisplay_show_str(const char *str, unsigned int delay_before, unsigned int delay_after) {
  if(Have_displays == 1) delay(delay_before);
  display_show_str(Altdisplay, str, delay_after);
}

void display_show_char(uint8_t dnum, uint8_t addr, unsigned char c) {
  display_show_char(dnum, addr, c, 0);
}
void display_show_char(uint8_t dnum, uint8_t addr, unsigned char c, unsigned int delay_after) {
  TM1637 *dp = NULL;
  if (dnum == 1) {
    #ifdef HAVE_DP1
    dp = &display1;
    #endif
  } else if (dnum == 2) {
    #ifdef HAVE_DP2
    dp = &display2;
    #endif
  }
  if (dp) dp->display(addr, c);
  if (delay_after) delay(delay_after);
}

//
// For the matrix display
//
void matrix_init(void) {
  MLEDScroll *mtx = NULL;
  #ifdef HAVE_MATRIX
  mtx = &matrix;
  #endif
  if (mtx) mtx->begin();
}

void matrix_clear(void) {
  MLEDScroll *mtx = NULL;
  #ifdef HAVE_MATRIX
  mtx = &matrix;
  #endif
  if (mtx) mtx->clear();
}

void matrix_brightness(int bri) {
  MLEDScroll *mtx = NULL;
  #ifdef HAVE_MATRIX
  mtx = &matrix;
  #endif
  if (bri < 0)
    bri = 0;
  else if (bri > 7)
    bri = 8;
  else
    bri = Mtx_bright[bri];
  if (mtx) mtx->setIntensity(bri); // 0 == OFF, 1-8
}

void matrix_icon(uint8_t i) {
  MLEDScroll *mtx = NULL;
  #ifdef HAVE_MATRIX
  mtx = &matrix;
  #endif
  if (mtx) mtx->icon(i);
}

void matrix_char(unsigned char c) {
  MLEDScroll *mtx = NULL;
  #ifdef HAVE_MATRIX
  mtx = &matrix;
  #endif
  if (mtx) mtx->character(c);
}

//
// Set internal LED on/off, but only if in use
//
void intled(uint8_t state) { intled(state, 0); }
void intled(uint8_t state, unsigned int delay_after) {
  #ifdef HAVE_INTLED
  digitalWrite(LED_BUILTIN, state);
  if (delay_after) delay(delay_after);
  #else
  (void)state;
  (void)delay_after;
  #endif
}

//
// Set intensity of all displays at once.
//
void set_brightness(void) { set_brightness(Brightness); }
void set_brightness(int bri) {
  if (bri < 0) {
    display_brightness(1, DEFAULT_BRIGHTNESS);
    display_brightness(2, DEFAULT_BRIGHTNESS);
    matrix_brightness(DEFAULT_BRIGHTNESS);
    Refresh = true;
    return;
  }
  if (Manual_bright > -1) {
    if (Debug->is_enabled())
      debug_log_ln("%s: Manual brightness override: %d (dp1 %d, dp2 %d, mtx %d)", TimeClient ? TimeClient->getDateTimeISO() : "Init", Manual_bright, Dp1_bright[Manual_bright], Dp2_bright[Manual_bright], Mtx_bright[Manual_bright]);
    bri = Manual_bright;
  }
  if ((Night_mode == NIGHT_MODE_ON) || (Night_mode && Luminance <= Night_lux)) {
    display_brightness(1, Night_bright[1]);
    display_brightness(2, Night_bright[2]);
    matrix_brightness(Night_bright[0]);
  } else {
    display_brightness(1, bri);
    display_brightness(2, bri);
    matrix_brightness(bri);
  }
  Refresh = true;
}

#ifdef HAVE_NMEA
//
// Parse GNSS $GxRMC sentence to globals
//
void parse_nmea_rmc(char *str) {
  double cog, sog;
  char *t;
  int i;
  t = strtok(str, ",");
  if (!t) return;  // incomplete RMC sentence
  for (i = 0; i < 7; i++) if (!(t = strtok(NULL, ","))) return; // incomplete
  sog = atof(t);
  if (!(t = strtok(NULL, ","))) return; // incomplete
  cog = atof(t);
  if (cog > 360 or sog > 100) return; // invalid (max SOG 100 kn - fix if needed...)
  NMEA_cog_d = cog;
  NMEA_sog_d = sog;
  if (!NMEA_mode) NMEA_mode = NMEA_init_mode;
  NMEA_ts = millis();
  if (NMEA_mode > 3) return; // nothing to display, why bother
  format_nmea_cog_sog();
  #ifdef DEBUG_NMEA_RMC
  debug_log_ln("RMC: COG %.2f, SOG %.2f %s [%.2f kn], M%d", cog, sog * Sog_multiplier[NMEA_sog_unit], Sog_unit_str[NMEA_sog_unit], sog, NMEA_mode);
  #endif
}
void format_nmea_cog_sog(void) {
  static const char *mu = "nh ";
  double sog_x;
  sog_x = NMEA_sog_d * 10.0f * Sog_multiplier[NMEA_sog_unit];
  if (Have_matrix) {
    if (Decimal_dp[1] && Decimal_dp[2]) {
      snprintf(NMEA_cog, 5, "%4.0f", NMEA_cog_d * 10);
      if (NMEA_cog_d < 1.0f) NMEA_cog[2] = '0';
      snprintf(NMEA_sog, 5, "%4.0f", sog_x);
      if (sog_x < 10.0f) NMEA_sog[2] = '0';
    } else {
      snprintf(NMEA_cog, 5, "%3.0f ", NMEA_cog_d);
      snprintf(NMEA_sog, 5, "%3.0f ", sog_x);
      if (sog_x < 10.0f) NMEA_sog[1] = '0';
    }
  } else {
    snprintf(NMEA_cog, 5, "%3.0fw", NMEA_cog_d); // 'w' maps to a degree sign, see TM1637.cpp
    snprintf(NMEA_sog, 5, "%3.0f%c", sog_x, mu[NMEA_sog_unit]);
    if (sog_x < 10.0f) NMEA_sog[1] = '0';
  }
}
#endif


//
// Main loop
//
void loop(void) {
  unsigned long rt = millis(); // For various timings incl. loop delay
  unsigned long looptime;
  static time_data tm;

  static int  prev_wday  = -2;    // display refresh comparison values
  static int  prev_min   = -2;    // --''--, -2 initially, 
  static int  prev_sec   = -2;    // --''--, -1 to force refresh later
  static int  prev_msec  = -2;    // --''--

  #ifdef HAVE_SWITCH
  static unsigned long prev_sw_ts = rt; // Previous switch state change millis()
  static int  prev_sw    =  0;    // switch state comparison value
  static int  sw_hot     =  0;    // hw switch has been closed for >n s
  #endif

  static bool do_lux         = true;  // do ambient light measurement
  static bool init_sync_ok   = false; // have synced time

  #ifdef HAVE_MATRIX
  static uint8_t mx_x        =  0; // previous/current matrix subsecond dot coordinates
  static uint8_t mx_y        =  0; // arrays of coordinates for a 1/24s clockwise dot:
  static uint8_t mx_xm_arr[] = { 4,5,6,7,7,7,7,7,7,6,5,4,3,2,1,0,0,0,0,0,0,1,2,3 };
  static uint8_t mx_ym_arr[] = { 0,0,0,1,2,3,4,5,6,7,7,7,7,7,7,6,5,4,3,2,1,0,0,0 };
  #endif

  #ifdef HAVE_NMEA
  static char nmea_curr[9] = "";
  static char nmea_prev[9] = "";
  #endif

  // Get the time (and resync the clock with the NTP server if that's due)
  if (TimeClient->update()) init_sync_ok = true;
  if (init_sync_ok) tm = TimeClient->parse_date_time();
  
  // MSec has changed and NOT in quiet mode or NMEA
  if (tm.MSec != prev_msec && !Quiet_mode && init_sync_ok) {
    if (Have_dp[2]) {
      char msbuf[8];
      snprintf(msbuf, sizeof(msbuf), "%02d", tm.MSec / 100);
      display_show_char(Dp[2], 2, msbuf[0], 0);
      display_show_char(Dp[2], 3, msbuf[1], 0);
    }
    #ifdef HAVE_MATRIX
    matrix.disBuffer[mx_y]^=(1<<(7-mx_x)); // flip previous 1/24s dot
    if (tm.Second)
      matrix.dot(4,0,0);
    else
      matrix.dot(4,0,1);
    mx_x = mx_xm_arr[(uint8_t)(tm.MSec/(10000/24))];
    mx_y = mx_ym_arr[(uint8_t)(tm.MSec/(10000/24))];
    matrix.disBuffer[mx_y]^=(1<<(7-mx_x)); // flip current 1/24s dot
    matrix.display();
    #endif // HAVE_MATRIX
    prev_msec = tm.MSec;
  }

  // Second has changed
  if (tm.Second != prev_sec && init_sync_ok) {
    if (!Quiet_mode) {
      char sbuf[8];
      snprintf(sbuf, sizeof(sbuf), "%02d", tm.Second);
      display_point(Dp[1], DpPc[Dp[1]]);
      display_point(Dp[2], DpPc[Dp[2]]);
      if (Have_dp[2]) {
        display_show_char(Dp[2], 0, sbuf[0], 0);
        display_show_char(Dp[2], 1, sbuf[1], 0);
      } else {
        display_show_char(Dp[1], 2, sbuf[0], 0);
        display_show_char(Dp[1], 3, sbuf[1], 0);
      }
      matrix_icon(tm.Second + MTX_SECMAP_OFFSET);
    } else { // quiet/NMEA mode
      #ifdef DEBUG_TIME_DISPLAY
      debug_log(".%02d", tm.Second); // NOTE: seconds checked first, so debug will show prev minute for hh:mm.00.
      #endif
      if (NMEA_mode != 2 && NMEA_mode != 3) {
        if ((tm.Second >= 55) && (!Night_mode || (Night_mode == NIGHT_MODE_AUTO && Luminance > Night_lux))) { 
          // Minute about to change, blink dot for 5 s at 1 Hz, unless in night mode
          if (Decimal_dp[1]) {
            // Set point bits 00000, 00010, 00110, 01110, 11110 for tm.Sec 55, 56, 57, 58, 59... :)
            display_point(Dp[1], ~(0xffff << (tm.Second - 54) & 31) & 30);
            #ifdef DEBUG_TIME_DISPLAY
            debug_log("!");
            #endif
          } else {
            display_point(Dp[1], (tm.Second % 2) ? POINT_OFF : DpPc[Dp[1]]);
            #ifdef DEBUG_TIME_DISPLAY
            debug_log("%s", (tm.Second % 2) ? "-" : ":");
            #endif
          }
          prev_min = -1; // Force update of display 1 for dot change to take effect
        } else if (Night_extreme && Night_mode && ((1UL << tm.Hour) & NiEx_h) && Luminance <= Night_lux && tm.Minute < 30) {
          display_point(Dp[1], POINT_OFF); // Just sets the flag
        } else {
          display_point(Dp[1], DpPc[Dp[1]]); // Just sets the flag
        }
      }
      if (Show_year) {
        prev_min = -1;
        Show_year--;
      }
    }
    check_wifi_status(); // Check WiFi status every second
    if (tm.Second % 10 == 1) check_ip_addr(); // Check IP address every 10s, notify of change
    do_lux = true; // Ambient light measurement every second
    prev_sec = tm.Second;
  }
  
  // Minute has changed
  if (tm.Minute != prev_min && init_sync_ok) {
    char mbuf[8];
    if (Night_extreme && Night_mode && ((1UL << tm.Hour) & NiEx_h) && Luminance <= Night_lux) {
      snprintf(mbuf, sizeof(mbuf), Time_format_niex[H_lead_zero], tm.Hour);
    } else {
      snprintf(mbuf, sizeof(mbuf), Time_format[H_lead_zero], tm.Hour, tm.Minute);
    }
    if ((Quiet_mode || Have_dp[2]) && NMEA_mode != 1) {
      display_show_str(Dp[1], mbuf);
      #ifdef DEBUG_TIME_DISPLAY
      debug_log("\n%02d:%02d", tm.Hour, tm.Minute);
      #endif
    }
    if (Quiet_mode && (NMEA_mode == 0 || NMEA_mode == 4)) {
      if (Have_dp[2]) { // update display 2 every minute, even if no changes, if WiFi is OK
        char dbuf[8] = "";
        if (Show_year) {
          snprintf(dbuf, sizeof(dbuf), "%04d", tm.Year);
          display_point(Dp[2], POINT_OFF);
        } else {
          snprintf(dbuf, sizeof(dbuf), DATEDISP_FORMAT, tm.Day, tm.Month);
          display_point(Dp[2], POINT_DATE);
        }
        if (WiFi.status() == WL_CONNECTED) display_show_str(Dp[2], dbuf);
      }
    } else if (!Have_dp[2] && (NMEA_mode == 0 || NMEA_mode == 5)) {
      // Not quiet mode, single display -> show minutes[:seconds]
      display_show_char(Dp[1], 0, mbuf[2], 0);
      display_show_char(Dp[1], 1, mbuf[3], 0);
    }
    if (tm.Wday != prev_wday) {
      if (Quiet_mode && (NMEA_mode == 0 || NMEA_mode == 4))
        matrix_icon(tm.Wday + Mtx_wd_idx[Mtx_wd_lc]);
      #ifdef HAVE_NMEA
      else if (NMEA_mode == 1) matrix_char(Swap_displays ? MTX_ICON_SOG_COG + NMEA_sog_unit : MTX_ICON_COG_SOG + NMEA_sog_unit);
      else if (NMEA_mode == 2) matrix_char(MTX_ICON_COG);
      else if (NMEA_mode == 3) matrix_char(MTX_ICON_SOG + NMEA_sog_unit);
      #endif
      // save the config (inc. drift comp) once a day, once we have enough NTP updates.
      if (TimeClient->getDriftSampleCount() >= 12 && prev_wday != -1) save_config();
      prev_wday = tm.Wday;
    }
    prev_min = tm.Minute;
  }

  #ifdef HAVE_NMEA
  if (NMEA_mode && NMEA_mode < 4) {
    snprintf(nmea_curr, 9, "%s%s", NMEA_cog, NMEA_sog);
    if (strncmp(nmea_curr, nmea_prev, 8)) {
      if (Have_matrix && Decimal_dp[1] && Decimal_dp[2]) {
        if (NMEA_mode == 1) {
          display_point(Dp[1], POINT_3);
          display_point(Dp[2], POINT_3);
          display_show_str(Dp[1], NMEA_cog);
          display_show_str(Dp[2], NMEA_sog);
        } else if (NMEA_mode == 2) {
          display_point(Dp[2], POINT_3);
          display_show_str(Dp[2], NMEA_cog);
        } else if (NMEA_mode == 3) {
          display_point(Dp[2], POINT_3);
          display_show_str(Dp[2], NMEA_sog);
        } // mode 4 time/date, mode 5 time/subsec
      } else {
        if (NMEA_mode == 1) {
          display_point(Dp[1], POINT_OFF);
          display_point(Dp[2], DpPc[Dp[2]]);
          display_show_str(Dp[1], NMEA_cog);
          display_show_str(Dp[2], NMEA_sog);
        } else if (NMEA_mode == 2) {
          display_point(Dp[2], POINT_OFF);
          display_show_str(Dp[2], NMEA_cog);
        } else if (NMEA_mode == 3) {
          display_point(Dp[2], DpPc[Dp[2]]);
          display_show_str(Dp[2], NMEA_sog);
        } // mode 4 time/date, mode 5 time/subsec
      }
      strcpy(nmea_prev, nmea_curr); // safe
    }
  }
  #endif

  #ifdef HAVE_SWITCH
  // Read switch state
  int sw_state = digitalRead(SWINPUT);
  if (sw_state != prev_sw) { // Switch state changed
    unsigned long ms_in_state = rt - prev_sw_ts; // Compare to loop start millis()
    if (Debug->is_enabled()) // may happen frequently, avoid unnecessary getDateTimeISO()
      debug_log_ln("%s: Switch %s after %lu ms", 
        TimeClient ? TimeClient->getDateTimeISO() : "Init", sw_state ? "closed" : "opened", ms_in_state);
    if (Have_intled) sw_state ? intled(ON) : intled(OFF); // LED on when sw closed
    if (!sw_state && ms_in_state >= 20 && !sw_hot) {
      // Toggle mode when switch released after 20 - 299 ms
      if (NMEA_mode) {
        #ifdef HAVE_NMEA
        NMEA_mode < 5 ? NMEA_mode++ : NMEA_mode=1;
        NMEA_init_mode = NMEA_mode;
        if (Debug->is_enabled())
          debug_log_ln("%s: NMEA mode set to %d: %s", TimeClient ? TimeClient->getDateTimeISO() : "Init", NMEA_mode, NMEA_mode_str[NMEA_mode]);
        if (NMEA_mode == 2 || NMEA_mode == 3) display_point(Dp[1], DpPc[Dp[1]]);
        #endif
      } else {
        if (Debug->is_enabled())
          debug_log_ln("%s: Display mode: %s", TimeClient ? TimeClient->getDateTimeISO() : "Init", !Quiet_mode ? "Normal" : "Seconds");
      }
      toggle_mode();
    }
    prev_sw_ts = rt; // Set as loop start millis()
    prev_sw    = sw_state;
    Refresh    = true;
    sw_hot     = 0;
  }
  if (sw_state) { // Switch remains closed (extended periods/functions)
    unsigned long ms_in_state = rt - prev_sw_ts; // millis() switch has been in state
    if (!sw_hot && ms_in_state >= 300) { 
      // NMEA sog unit cycle when switch closed for 300 - 599 ms
      #ifdef HAVE_NMEA
      if (NMEA_mode == 1 || NMEA_mode == 3) {
        NMEA_sog_unit++; if (NMEA_sog_unit > 2) NMEA_sog_unit = 0;
        if (Debug->is_enabled())
          debug_log_ln("%s: SOG unit set to %s", TimeClient ? TimeClient->getDateTimeISO() : "Init", Sog_unit_str[NMEA_sog_unit]);
      }
      Refresh = true;
      #endif
      sw_hot = 1;
    } 
    if (sw_hot == 1 && ms_in_state >= 600) { 
      #ifdef HAVE_NMEA
      // Swap displays when switch closed for 600 - 1199 ms
      if (NMEA_mode == 1 || NMEA_mode == 3) {
        NMEA_sog_unit--; if (NMEA_sog_unit < 0) NMEA_sog_unit = 2; // revert unwanted unit cycle
        if (Debug->is_enabled())
          debug_log_ln("%s: SOG unit revert to %s", TimeClient ? TimeClient->getDateTimeISO() : "Init", Sog_unit_str[NMEA_sog_unit]);
      }
      if (NMEA_mode) {
        display_swap();
      } else {
        cycle_night_mode(1);
      }
      #else
      // Cycle Night Mode
      cycle_night_mode(1);
      #endif
      sw_hot = 2;
    } 
    if (sw_hot == 2 && ms_in_state >= 1200) { 
      // Show IP addr when switch closed for 1,2s+
      #ifdef HAVE_NMEA
      // revert unwanted swap
      if (NMEA_mode) {
        display_swap();
      } else {
        cycle_night_mode(-1);
      }
      #else
      cycle_night_mode(-1);
      #endif
      display_point(1, POINT_OFF);
      display_ip_addr(0);
      sw_hot = 3;
    }
    if (sw_hot == 3 && ms_in_state >= 3600) { 
      // Reboot when switch closed for 3,6s+
      debug_log_ln("Reboot requested via switch");
      display_show_str(1, "boot", 500);
      ESP.restart();
    }
  }
  #endif // HAVE_SWITCH

  // Ambient light level check
  if (do_lux) {
    check_luminosity();
    do_lux = false;
  }

  #ifdef HAVE_NMEA
  // NMEA timeout
  if (NMEA_mode && ((millis() - NMEA_ts) > NMEA_TIMEOUT)) {
    if (Debug->is_enabled())
      debug_log_ln("%s: NMEA timeout, mode set to 0", TimeClient ? TimeClient->getDateTimeISO() : "Init");
    NMEA_mode = 0;
    Quiet_mode = true;
    if (Swap_displays)
      display_swap(false);
    Refresh = true;
  }
  #endif
  
  // Force refresh of all displays
  if (Refresh) {
    prev_wday = -1; prev_min = -1; prev_sec = -1; prev_msec = -1;
    #ifdef HAVE_NMEA
    nmea_prev[0] = '\0';
    format_nmea_cog_sog();
    #endif
    Refresh = false;
  }

  // Handle any pending requests over the serial interface
  check_serial_input();

  // Handle any pending web client
  Httpd.handleClient();

  // Handle any pending over the air updates.
  ArduinoOTA.handle();

  // Small loop delay (results in a max. 100 Hz loop frequency)
  looptime = millis() - rt;
  if (looptime < 10) delay(10 - looptime);
}

// EOF //
