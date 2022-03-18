#pragma once
#include "Arduino.h"
#include <Udp.h>
#include "Debug.h"

#define DEBUG_NTP_DRIFT           // initial value, can be set dynamically
#define DEBUG_NTP_PACKETS         // initial value, can be set dynamically
#undef  DEBUG_NTP_PARSE_DATE_TIME // for DST calculation etc. debugging, compile-time

#define NTP_LOCAL_PRECISION         -15
#define NTP_VERSION                   4
#define NTP_PORT                    123
#define NTP_DEFAULT_LOCAL_PORT     1337
#define NTP_PACKET_READ_DELAY_MS      2
#define NTP_PACKET_READ_ATTEMPTS    250
#define NTP_INITIAL_XMT    1500000000UL // if our clock is 0, use this. Fri Jul 14 02:40:00 2017 UTC

#define MS_IN_H        3600000UL // ms in one hour (used for debug output formatting)
#define DEF_POLL       3600000UL // default NTP server poll frequency, ms
#define MIN_POLL         15000UL // minimum NTP server poll frequency, ms - never set lower than 15 s
#define INIT_POLL_DIVIDE      32 // divide poll freq initially to get semi-ok time faster (still never less than MIN_POLL) (default 1h / 2^7 = 28s)
#define DRIFT_MS_PER_H_OK    250 // always accept to drift table if less

#define DRIFT_ARRAY_SIZE      12 // Number of drift measurements to average
#define DRIFT_MS_PER_H_MAX  2000 // If more than n ms/h, exclude sample from averaging
#define MAX_DRIFT_MULTIPLIER 3.0 // If last measured drift per second exceeds the stored 
                                 // average _driftComp * this, exclude sample from averaging

#define MAX_ISO_TS 26            // ISO date-time string storage size

// Timestamp conversion macroni
typedef uint64_t tstamp;         // NTP timestamp format
typedef uint32_t tdist;          // NTP short format

#define JAN_1970  2208988800ULL  // NTP time to unixtime offset, ie. 1900-01-01T00:00
#define FRIC            65536.0            // 2^16 as a double
#define FRAC       4294967296.0            // 2^32 as a double
#define D2FP(r)    ((tdist)((r) * FRIC))   // NTP short
#define FP2D(r)    ((double)(r) / FRIC)
#define D2LFP(a)   ((tstamp)((a) * FRAC))  // NTP timestamp
#define LFP2D(a)   ((double)(a) / FRAC)
#define LFP2DS(a)  ((double)(int64_t)(a) / FRAC)
#define LOG2D(a)   ((a) < 0 ? 1. / (1L << -(a)) : 1L << (a)) // poll, etc.
#define PHI        15e-6                   // % frequency tolerance (15 ppm)

#define U2LFP(a)   (((uint64_t)((a).tv_sec + JAN_1970) << 32) \
                   + (uint64_t)((a).tv_usec / 1e6 * FRAC))

#ifndef min
#define min(a, b)  ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b)  ((a) < (b) ? (b) : (a))
#endif


extern "C" {
  // Signature for our callback functions(s)
  typedef void (*callbackFunction)(void);

  // time-data as a structure.
  typedef struct {
    int MSec;
    int Second;
    int Minute;
    int Hour;
    int Wday;
    int Day;
    int Month;
    int Year;
    int IsDST;
  } time_data;

  // NTP packet structure
  typedef struct {
    uint8_t   li_vn_mode;     // leap indicator, version and mode
    uint8_t   stratum;        // peer stratum
    uint8_t   ppoll;          // peer poll interval
    int8_t    precision;      // peer clock precision
    uint32_t  rootdelay;      // distance to primary clock
    uint32_t  rootdispersion; // clock dispersion
    uint32_t  refid;          // reference clock ID
    uint64_t  ref;            // time peer clock was last updated
    uint64_t  org;            // originate time stamp
    uint64_t  rec;            // receive time stamp
    uint64_t  xmt;            // transmit time stamp
  } ntp_packet;

  // NTP support variables
  typedef struct {
    double  xmt;
    double  dst;
    double  offset;
    double  ndelay;
    double  disprs;
    double  epoc;
    uint64_t us; // micros64() at time epoch stored
    unsigned long seq; // sequence number
  } ntp_vars;
}


class NTPClient {
  private:
    UDP*          _udp;
    bool          _udpSetup       = false;

    const char*   _poolServerName = "pool.ntp.org"; // Default time server
    int           _port           = NTP_DEFAULT_LOCAL_PORT;
    float         _timeOffset     = 0.0;
    int           _eu_dst         = 0;

    unsigned long _updateInterval = DEF_POLL;  // NTP request interval in ms
    unsigned int  _updateDivisor  = INIT_POLL_DIVIDE; // Divide interval by x at start, to
                                               // get an approximate drift value faster
                                               // Don't go too low, causes very bad estimates.

    uint64_t      _lastUpdate64   = 0;         // Last update time, micros64()
    unsigned long _lastUpdate     = 0;         // Last update time, millis()
    unsigned long _lastAttempt    = 0;         // Last request time, millis()
    unsigned long _lastErr        = 0;         // Last failed request time,  millis()
    double        _lastEpoc       = NTP_INITIAL_XMT; // Epoch time returned by the last NTP request
    double        _lastErrEpoc    = NTP_INITIAL_XMT; // Epoch time when last NTP request failed
    double        _driftComp      = 0.0;       // Clock drift compensation in s/s
    unsigned long _driftSamples   = 0;         // Count of accepted drift measurements
    double        _driftStats[DRIFT_ARRAY_SIZE]; // Latest accepted clock drift measurements, rotates
    double        _fnerrStats[DRIFT_ARRAY_SIZE]; // Final error statistics, rotates
    double        _dcompStats[DRIFT_ARRAY_SIZE];  // Compensation value stats, rotates

    unsigned long _updateCount    = 0;         // number of successful NTP requests
    unsigned long _errorCount     = 0;         // number of unsuccessful NTP requests

    #ifdef DEBUG_NTP_PACKETS
    bool          _debugPackets   = true;
    #else
    bool          _debugPackets   = false;
    #endif
    #ifdef DEBUG_NTP_DRIFT
    bool          _debugDrift     = true;
    #else
    bool          _debugDrift     = false;
    #endif

    void          initNTPPacket           (ntp_packet *pkt);
    bool          sendNTPPacket     (const ntp_packet *pkt);
    uint64_t      readNTPPacket           (ntp_packet *pkt,       ntp_vars *nv);
    bool          validateNTPPacket (const ntp_packet *pkt, const ntp_vars *nv);
    void          calculateNTPOffset(const ntp_packet *pkt,       ntp_vars *nv);
    void          debugNTPPacket    (const ntp_packet *pkt, const ntp_vars *nv);
    bool          check_update_drift(const ntp_vars *nv);
    void          flushUDPPackets(void);

    // 32-bit byte endianness swap
    uint32_t      jk_ntohl(const uint32_t n);
    uint32_t      jk_htonl(const uint32_t n);

    // 64-bit byte endianness swap
    uint64_t      jk_ntohll(const uint64_t n);
    uint64_t      jk_htonll(const uint64_t n);

    // Callback handles.
    callbackFunction on_before = NULL;
    callbackFunction on_after  = NULL;

    // time-data structure storage, NOT guaranteed to be current time
    // use parse_date_time() to set.
    time_data _data;

  public:
    NTPClient(UDP& udp);
    NTPClient(UDP& udp, float timeOffset);
    NTPClient(UDP& udp, const char* poolServerName);
    NTPClient(UDP& udp, const char* poolServerName, float timeOffset);
    NTPClient(UDP& udp, const char* poolServerName, float timeOffset, unsigned long updateInterval);

    // Invoke this user-function before we update.
    void on_before_update(callbackFunction newFunction);

    // Invoke this user-function after we update.
    void on_after_update(callbackFunction newFunction);

    // Starts the underlying UDP client with the default local port
    void begin(void);

    // Starts the underlying UDP client with the specified local port
    void begin(int port);

    // Stops the underlying UDP client
    void end(void);

    // This should be called frequently in the main loop of your application.
    // returns true on NOP and success, false on failure to sync
    bool update(void);

    // force update from the NTP Server
    // returns true on success, false on failure
    bool forceUpdate(void);

    // Return time-data as a structure.
    time_data parse_date_time(void);  // defaults to this-getEpochTime()
    time_data parse_date_time(const double entryTime);  // unixtime.ns

    // Change the time server.
    void setNtpServer(const char *server);
    void setNtpServer(const char *server, int port);

    // Changes the time offset.
    void setTimeOffset(const float timeOffset);

    // Set EU DST rule conformance on/off
    void setEUDST(const int eu_dst);
  
    // Get/set the update interval.
    unsigned long getUpdateInterval(void); // true poll frequency = interval/divisor
    unsigned int  getUpdateDivisor(void);  // divisor settles to 1 after a few cycles
    void          setUpdateInterval(const unsigned long updateInterval); // in ms

    // Get/Set the initial drift compensation value in ms/h (human-readable)
    double getDriftComp(void);
    void   setDriftComp(const double driftcomp);
    
    // Get all stored drift values in ms/h as string of max maxlen chars stored to buf,
    // provide a printf format string with %f somewhere as format.
    // Returns the number of stored drift samples or < 0 for a failure (buffer too small)
    int getDriftValues(char *buf, const char *format, size_t maxlen);

    // Get Daylight Saving Time status, returns 0 (normal time), 1 (summer time) or -1 (DST rules not used).
    int isDST(void); // uses getEpochTime()
    int isDST(const double epochtime);

    // Get formatted time string from unixtime.ns value
    // If called with *s, the target char array needs to have room for at least 25 chars.
    void  getDateTimeISO(char *s);                // writes to s, uses getEpochTime()
    void  getDateTimeISO(char *s, const double epochtime); // writes to s
    char *getDateTimeISO(void);                   // uses getEpochTime(), local buffer
    char *getDateTimeISO(const double epochtime); // uses a local static char buffer
    void  getTime(char *s);                       // writes to s, uses getEpochTime()
    void  getTime(char *s, const double epochtime); // writes to s
    char *getTime(void);                          // uses getEpochTime(), local buffer
    char *getTime(const double epochtime);        // uses a local static char buffer

    // Get current unixtime.ns
    double getEpochTime(void); // compensated for drift
    double getEpochRaw(void);  // uncompensated

    // Get time of last NTP sync
    double        getLastUpdateEpoch(void);  // in unixtime.ns
    unsigned long getLastUpdateMillis(void); // in millis()
    double        getLastErrEpoch(void);     // last failure in unixtime.ns
    unsigned long getLastErrMillis(void);    // last failure in millis()
    double        getNextUpdateEpoch(void);  // in unixtime.ns
    unsigned long getNextUpdateMillis(void); // in millis()
    char         *getNextUpdateTime(bool with_date); // stored string
    char         *getNextUpdateTime(void);   // without date

    // Get statistics 
    unsigned long getUpdateCount(void);      // number of NTP updates since start
    unsigned long getErrorCount(void);       // number of failed NTP updates
    unsigned long getDriftSampleCount(void); // number of accepted drift samples

    // Set debug flags 
    void setDebugPackets(bool state);
    void setDebugDrift(bool state);
    
};
