/**
 * The MIT License (MIT)
 * Copyright (c) 2015 by Fabrice Weinberg
 * 
 * Extensively modified/rewritten by Janne Korkkula <jk@iki.fi> 2018-2019
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <ESP8266WiFi.h>
#include "NTPClient.h"

#ifndef LEAP_YEAR
#  define LEAP_YEAR(Y)     ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ) )
#endif

NTPClient::NTPClient(UDP& udp) {
  this->_udp            = &udp;
}
NTPClient::NTPClient(UDP& udp, float timeOffset) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
}
NTPClient::NTPClient(UDP& udp, const char* poolServerName) {
  this->_udp            = &udp;
  this->_poolServerName = poolServerName;
}
NTPClient::NTPClient(UDP& udp, const char* poolServerName, float timeOffset) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
}
NTPClient::NTPClient(UDP& udp, const char* poolServerName, float timeOffset, unsigned long updateInterval) {
  this->_udp            = &udp;
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
  this->_updateInterval = updateInterval;
}

void NTPClient::on_before_update( callbackFunction newFunction ) {
  this->on_before = newFunction;
}
void NTPClient::on_after_update( callbackFunction newFunction ) {
  this->on_after = newFunction;
}

void NTPClient::begin(void) {
  this->begin(NTP_DEFAULT_LOCAL_PORT);
}

void NTPClient::begin(int port) {
  this->_port = port;
  this->_udp->begin(this->_port); // LOCAL port...
  this->_udpSetup = true;
  this->_updateInterval = max(this->_updateInterval, MIN_POLL); 
}

void NTPClient::end(void) {
  this->_udp->stop();
  this->_udpSetup = false;
}

bool NTPClient::update(void) {
  if (( 
        (millis() - this->_lastUpdate) >= (this->_updateInterval / this->_updateDivisor)
          || 
        this->_lastUpdate == 0
      ) && (
        (millis() - this->_lastAttempt) >= MIN_POLL
          || 
        this->_lastAttempt == 0
      )) 
  {
    if (!this->_udpSetup) this->begin(); // Setup the UDP client if needed
    return this->forceUpdate();
  }
  return this->_updateCount > 0 ? true : false; // NOP
}

bool NTPClient::forceUpdate(void) {
  uint64_t new_update = 0; // new NTP update timestamp (~micros64)
  unsigned long send_us    = 0; // micros() for rtt/offset timing
  unsigned long prev_err   = this->_lastErr; 
  double prev_err_epoc     = this->_lastErrEpoc;

  static bool noconn_err = false;
  bool is_valid;

  static ntp_packet msg_s;
  ntp_packet *msg = &msg_s;

  static ntp_vars   ntp_vars_s = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0 };
  ntp_vars *nv    = &ntp_vars_s;

  char buf[256];

  // errors reset before successful return: 'if (err) return false;' == OK
  this->_errorCount++;
  this->_lastAttempt = millis();
  this->_lastErr = this->_lastAttempt;
  this->_lastErrEpoc = getEpochTime();

  // Check WiFi status
  if (WiFi.status() != WL_CONNECTED) {
    if (Debug->is_enabled() && !noconn_err) debug_log_ln("Warning: WiFi not connected, NTP sync skipped");
    noconn_err = true;
    return false;
  } else {
    noconn_err = false;
  }

  // pre-request-hook
  if (on_before) on_before();

  // clear any late arrivals (WiFiUDP flush() seems useless?)
  flushUDPPackets();

  initNTPPacket(msg);
  if (!this->_updateCount) {
    // send initial packet
    nv->xmt  = this->getEpochRaw() + JAN_1970;
    msg->xmt = this->jk_htonll(D2LFP(nv->xmt));
    send_us  = micros();
    if (!sendNTPPacket(msg)) return false;
    if (!(new_update = this->readNTPPacket(msg, nv))) return false; // sets nv->us
    nv->dst = nv->xmt + ((double)(micros() - send_us) / 1e6);
    //nv->dst = this->getEpochRaw() + ((micros() - send_us) % 1000 / 1000) + JAN_1970;
  } else {
    // following packets
    msg->rec = this->jk_htonll(D2LFP(nv->dst));
    msg->org = msg->xmt;
    nv->xmt  = nv->epoc + ((double)(micros64() - nv->us) / 1e6) + JAN_1970;
    msg->xmt = this->jk_htonll(D2LFP(nv->xmt));
    if (!sendNTPPacket(msg)) return false;
    if (!(new_update = this->readNTPPacket(msg, nv))) return false; // sets nv->dst, nv->us
  }

  is_valid = validateNTPPacket(msg, nv);
  calculateNTPOffset(msg, nv); // sets nv->epoc etc.
  if (this->_debugPackets && Debug->is_enabled()) debugNTPPacket(msg, nv);
  if (!is_valid) return false;
  if (this->_updateCount) this->check_update_drift(nv);

  // all done, set "system time"
  this->_lastUpdate   = millis();
  this->_lastUpdate64 = nv->us;
  this->_lastEpoc     = nv->epoc;
  this->_updateCount++;
  nv->seq++;

  if(Debug->is_enabled()) {
    snprintf(buf, sizeof(buf), 
      "NTP update %lu OK, next ~%s", 
      this->_updateCount, getNextUpdateTime());
    debug_log_ln(buf);
  }

  // post-update-hook
  if (on_after) on_after();

  // success, clear preset error state
  this->_errorCount--;
  this->_lastErr     = prev_err;
  this->_lastErrEpoc = prev_err_epoc;
  return true;
}

void NTPClient::initNTPPacket(ntp_packet *pkt) {
  pkt->li_vn_mode     = 0b11 << 6 | NTP_VERSION << 3 | 3;
  pkt->stratum        = 0;
  pkt->ppoll          = 4;
  pkt->precision      = 0;
  pkt->rootdelay      = 0;
  pkt->rootdispersion = 0;
  pkt->refid          = 0;
  pkt->ref            = 0;
  pkt->org            = 0;
  pkt->rec            = 0;
}

bool NTPClient::sendNTPPacket(const ntp_packet *pkt) {
  if(!this->_udp->beginPacket(this->_poolServerName, NTP_PORT)) {
    debug_log_ln("Warning: Connecting to NTP server failed");
    return false;
  }
  this->_udp->write((byte *)pkt, sizeof(ntp_packet));
  if (!this->_udp->endPacket()) {
    debug_log_ln("Warning: Sending request to NTP server failed");
    return false;
  }
  return true;
}

uint64_t NTPClient::readNTPPacket(ntp_packet *pkt, ntp_vars *nv) {
  // Wait until data is there or timeout.
  uint64_t new_update; // new NTP update timestamp (~micros64()), return value
  unsigned long start;
  unsigned int i = 0;
  unsigned int c = 0;

  start = micros();
  do {
    delay (NTP_PACKET_READ_DELAY_MS);
    c = this->_udp->parsePacket();
    if (i > NTP_PACKET_READ_ATTEMPTS) {
      debug_log_ln("Warning: Request sent to NTP server timed out after %d ms", 
        NTP_PACKET_READ_DELAY_MS * NTP_PACKET_READ_ATTEMPTS);
      return 0;
    }
    i++;
  } while (!c);

  // Compensate for the delay waiting for the return packet to arrive
  new_update = micros64() - (micros() - start);
  if (nv->seq) nv->dst = nv->epoc + (new_update - nv->us) / 1e6 + JAN_1970;
  nv->us = new_update;

  if (c > sizeof(ntp_packet) && this->_debugPackets) debug_log_ln("Warning: readNTPPacket(): %u bytes in UDP rx queue", c); 
  do {
    this->_udp->read((byte *)pkt, sizeof(ntp_packet));
    c -= sizeof(ntp_packet);
  } while (c >= sizeof(ntp_packet));

  return new_update;
}

bool NTPClient::validateNTPPacket(const ntp_packet *pkt, const ntp_vars *nv) {
  uint64_t xmt_n = this->jk_htonll(D2LFP(nv->xmt));
  if(pkt->li_vn_mode >> 6 == 0b11) {
    if(this->_updateCount) {
      debug_log_ln("Warning: NTP server not synchronized, ignoring packet");
      return false;
    } else {
      debug_log_ln("Warning: NTP server not synchronized, only using for initial sync");
    }
  }
  if(!(pkt->li_vn_mode & 4)) {
    debug_log_ln("Warning: NTP packet invalid (mode != server)");
    return false;    
  }
  if(jk_ntohl(pkt->stratum) == 0) {
    switch (jk_ntohl(pkt->refid)) {
      case 0x44454e59 : debug_log_ln("Warning: NTP server denied access (DENY)");           break; // DENY
      case 0x52535452 : debug_log_ln("Warning: NTP access restricted by policy (RSTR)");    break; // RSTR
      case 0x52415445 : debug_log_ln("Warning: NTP server poll frequency too high (RATE)"); break; // RATE
      default         : debug_log_ln("Warning: NTP packet invalid (stratum == 0)");
    }
    return false;
  }
  if(jk_ntohll(pkt->xmt) == 0) {
    debug_log_ln("Warning: NTP packet invalid (xmt == 0), ignoring");
    return false;    
  }
  if(pkt->org != xmt_n) {
    debug_log_ln("Warning: NTP packet invalid (org != out), ignoring");
    return false;    
  }
  if(pkt->xmt == xmt_n) {
    debug_log_ln("Warning: NTP packet invalid (xmt == out), ignoring");
    return false;    
  }
  return true;
}

void NTPClient::calculateNTPOffset(const ntp_packet *pkt, ntp_vars *nv) {
  nv->offset = ( LFP2DS( this->jk_ntohll(pkt->rec) - this->jk_ntohll(pkt->org) ) +
                 LFP2DS( D2LFP(nv->dst) - this->jk_ntohll(pkt->xmt) ) ) / 2;

  nv->ndelay = max(  LFP2DS( D2LFP(nv->dst) - this->jk_ntohll(pkt->org) )
                   - LFP2DS( this->jk_ntohll(pkt->rec) - this->jk_ntohll(pkt->xmt) ),
                     LOG2D(NTP_LOCAL_PRECISION) );

  nv->disprs = LOG2D(pkt->precision) + LOG2D(NTP_LOCAL_PRECISION) + PHI *
    LFP2DS( D2LFP(nv->dst) - this->jk_ntohll(pkt->org) );

  // set to NTP server time plus measured delay
  nv->epoc = LFP2D(this->jk_ntohll(pkt->xmt)) + nv->ndelay - JAN_1970;
}

void NTPClient::debugNTPPacket(const ntp_packet *pkt, const ntp_vars *nv) {
  char debug_ntp_r[1024];
  snprintf(debug_ntp_r, sizeof(debug_ntp_r), 
    "NTP packet %lu: stratum %d, precision %d, refid %#x [%u.%u.%u.%u]\r\n"
    "out: %20.9f   ref: %20.9f\r\n"
    "org: %20.9f      offset:%15.9f\r\n"
    "rec: %20.9f       delay:%15.9f\r\n"
    "xmt: %20.9f  dispersion:%15.9f\r\n"
    "dst: %20.9f\r\n",
    nv->seq + 1, pkt->stratum, pkt->precision, jk_ntohl(pkt->refid), 
       pkt->refid&0xff,pkt->refid>>8&0xff,pkt->refid>>16&0xff,pkt->refid>>24,
    nv->xmt, LFP2D(this->jk_ntohll(pkt->ref)), 
    LFP2D(this->jk_ntohll(pkt->org)), nv->offset,
    LFP2D(this->jk_ntohll(pkt->rec)), nv->ndelay,
    LFP2D(this->jk_ntohll(pkt->xmt)), nv->disprs,
    nv->dst
  );
  debug_log(debug_ntp_r);
}

double NTPClient::getDriftComp(void) {
  return this->_driftComp * MS_IN_H;
}
void NTPClient::setDriftComp(const double driftcomp) {
  double init_drift = driftcomp / MS_IN_H * -1; // -1: drift, not compensation value.
  if (init_drift == 0.0) {
    // CLEAR drift table
    if(this->_debugDrift)
      debug_log_ln("Clearing drift table");
    for (uint8_t i = 0; i < DRIFT_ARRAY_SIZE; i++) {
      this->_driftStats[i] = 0.0;
      this->_fnerrStats[i] = 0.0;
      this->_dcompStats[i] = 0.0;
    }
    this->_driftSamples  =   0;
    this->_driftComp     = 0.0;
    this->_updateCount   =   0; // necessary to clear ALL history
  } else {
    if(this->_debugDrift && Debug->is_enabled()) {
      char debug_drift_init[256] = "";
      snprintf(debug_drift_init, sizeof(debug_drift_init), 
        "Seeding drift table with compensation value %+.2f ms/h (%+.4f µs/s)\r\nSetting poll frequency divisor to 1",
        init_drift * MS_IN_H * -1, init_drift * 1000000L * -1);
      debug_log_ln(debug_drift_init);
    }
    this->_driftStats[0] = init_drift;
    this->_fnerrStats[0] = 0.0;
    this->_driftComp     = init_drift * -1;
    this->_dcompStats[0] = init_drift * -1;
    this->_driftSamples  = 1;
    this->_updateDivisor = 1; // disable initial sync speedup
  }
}

int NTPClient::getDriftValues(char *buf, const char *format, size_t maxlen) {
  int c = maxlen - 1;
  int stored_drift_samples = min(this->_driftSamples, DRIFT_ARRAY_SIZE);
  int j = (this->_driftSamples < DRIFT_ARRAY_SIZE) ? this->_driftSamples - 1 : (this->_driftSamples - 1) % DRIFT_ARRAY_SIZE;
  buf[0] = 0; // clear output
  if (this->_driftSamples < 1) return 0; // no samples yet
  for (int i = 0; i < stored_drift_samples && c > 0; i++) {
    char ss[128];
    snprintf(ss, sizeof(ss), format, (this->_driftStats[j] * MS_IN_H), (this->_dcompStats[j] * MS_IN_H), (this->_fnerrStats[j] * 1000));
    if (j) j--; else j = DRIFT_ARRAY_SIZE - 1;
    c -= strlen(ss); // "wrong" order to get full lines
    if (c > 0) strncat(buf, ss, c);
  }
  if (c < 0) return c;
  return stored_drift_samples;
}

bool NTPClient::check_update_drift(const ntp_vars *nv) {
  bool is_valid_update = true; // default to OK
  double new_driftcomp = this->_driftComp;
  char debug_drift_warn[128] = "";
  char debug_drift[256]      = "";

  double since_update_s      = (double)(nv->us - this->_lastUpdate64) / 1e6;
  double since_update_corr_s = since_update_s + this->_driftComp * since_update_s;
  double prev_epoc           = this->_lastEpoc + since_update_s;
  double prev_epoc_corrected = this->_lastEpoc + since_update_corr_s;
  double drift_abs_s         = prev_epoc - nv->epoc;
  double drift_per_s         = drift_abs_s / since_update_corr_s;
  double drift_change        = fabs(this->_driftComp * -1 - drift_per_s);
  double comp_error_abs_s    = prev_epoc_corrected - nv->epoc;
  double comp_error_per_s    = comp_error_abs_s / since_update_corr_s;

  // drift calculations start from round 2
  if (!this->_updateCount) return true;

  if (this->_driftSamples == 0) {
    // second packet, first measurement
    this->_driftSamples  = 1;
    this->_driftStats[0] = drift_per_s;
    this->_dcompStats[0] = this->_driftComp;
    this->_fnerrStats[0] = comp_error_abs_s;
    new_driftcomp        = drift_per_s * -1;
  } else {
    // 2 or more drift measurements
    // if starting and not horrible, or delta less than avg * MAX_.., or both the average and sample < .._OK
    if ((    (fabs(drift_per_s) * MS_IN_H <= DRIFT_MS_PER_H_MAX)
          && (this->_driftSamples <= DRIFT_ARRAY_SIZE / 3) 
        ) || (drift_change <= fabs(this->_driftComp) * MAX_DRIFT_MULTIPLIER)
          || (     (fabs(this->_driftComp) * MS_IN_H <= DRIFT_MS_PER_H_OK)
                && (fabs(drift_per_s) * MS_IN_H <= DRIFT_MS_PER_H_OK) 
              ) 
    ) { // drift within absolute limit && (too few updates to filter || delta within limit)
      this->_driftStats[this->_driftSamples   % DRIFT_ARRAY_SIZE] = drift_per_s;
      this->_dcompStats[this->_driftSamples   % DRIFT_ARRAY_SIZE] = this->_driftComp;
      this->_fnerrStats[this->_driftSamples++ % DRIFT_ARRAY_SIZE] = comp_error_abs_s;
      // set compensation value as average of drift table
      double drift_sum = 0.0;
      for (uint8_t i = 0; i < min(this->_driftSamples, DRIFT_ARRAY_SIZE); i++) {
        drift_sum += this->_driftStats[i];
      }
      new_driftcomp = drift_sum / min(this->_driftSamples, DRIFT_ARRAY_SIZE) * -1;
    } else {
      // drift exceeds limits
      if(this->_debugDrift && Debug->is_enabled()) {
        snprintf(debug_drift_warn, sizeof(debug_drift_warn), 
          "WARNING: drift > %.2f*avg or > %d ms/h, not updating correction table\r\n",
          MAX_DRIFT_MULTIPLIER, DRIFT_MS_PER_H_MAX);
      }
      is_valid_update = false;
    }
  }

  if(this->_debugDrift && Debug->is_enabled()) {
    snprintf(debug_drift, sizeof(debug_drift), 
      "Update %lu, interval %.3f s (~%.3f s), set at %lu s / %d\r\n"
      "Unixtime %.5f (~%.5f) => %.5f\r\n",
      this->_updateCount + 1, since_update_s, since_update_corr_s, this->_updateInterval / 1000, this->_updateDivisor,
      prev_epoc, prev_epoc_corrected, nv->epoc);
    debug_log(debug_drift);
    snprintf(debug_drift, sizeof(debug_drift), 
      "Clock drift:%+8.2f ms %+10.3f µs/s %+8.1f ms/h\r\n"
      "Final error:%+8.2f ms %+10.3f µs/s %+8.1f ms/h\r\n",
      drift_abs_s      * 1000, drift_per_s       * 1e6, drift_per_s      * MS_IN_H,
      comp_error_abs_s * 1000, comp_error_per_s  * 1e6, comp_error_per_s * MS_IN_H);
    debug_log(debug_drift);
    snprintf(debug_drift, sizeof(debug_drift), 
      "Drift correction delta: %+10.3f µs/s %+8.1f ms/h\r\n"
      "Drift correction value: %+10.3f µs/s %+8.1f ms/h (%lu samples)\r\n",
      (new_driftcomp - this->_driftComp) * 1e6, (new_driftcomp - this->_driftComp) * MS_IN_H,
      new_driftcomp * 1e6, new_driftcomp * MS_IN_H, min(this->_driftSamples, DRIFT_ARRAY_SIZE));
    debug_log(debug_drift);
    if(strlen(debug_drift_warn) > 0) debug_log(debug_drift_warn);
  }

  if(!is_valid_update) 
    return false;

  // after initial request, halve poll frequency divider until at 1
  if (this->_updateDivisor > 1 && this->_updateCount)
    this->_updateDivisor /= 2;

  // store the updated drift compensation value
  this->_driftComp = new_driftcomp;

  return true;
}

double NTPClient::getEpochRaw(void) {
  return this->_lastEpoc + (double)(micros64() - this->_lastUpdate64) / 1e6; 
  // Uncompensated time since last NTP update
}

double NTPClient::getEpochTime(void) {
  double s_since_update = (double)(micros64() - this->_lastUpdate64) / 1e6;
  return this->_lastEpoc + s_since_update + (s_since_update * this->_driftComp); 
  // Drift-compensated time since last NTP update 
}

double NTPClient::getLastUpdateEpoch(void) {
  return this->_lastEpoc; 
}
unsigned long NTPClient::getLastUpdateMillis(void) {
  return this->_lastUpdate; 
}
double NTPClient::getLastErrEpoch(void) {
  return this->_lastErrEpoc; 
}
unsigned long NTPClient::getLastErrMillis(void) {
  return this->_lastErr;
}
double NTPClient::getNextUpdateEpoch(void) {
  unsigned long w = this->_updateInterval / this->_updateDivisor;
  unsigned long i = (w >= MIN_POLL ? w / 1000 : MIN_POLL / 1000);
  return this->_lastEpoc + i;
}
unsigned long NTPClient::getNextUpdateMillis(void) {
  unsigned long w = this->_updateInterval / this->_updateDivisor;
  unsigned long i = (w >= MIN_POLL ? w : MIN_POLL);
  return this->_lastUpdate + i;
}
char *NTPClient::getNextUpdateTime(void) {
  return this->getNextUpdateTime(false);
}
char *NTPClient::getNextUpdateTime(bool with_date) {
  static char buf[72];
  unsigned long w = this->_updateInterval / this->_updateDivisor;
  unsigned long i = (w >= MIN_POLL ? w / 1000 : MIN_POLL / 1000);
  snprintf(buf, sizeof(buf), 
    "%s, in %lu s, rate %lu s (%.2f h) / %u",
    (with_date ? getDateTimeISO(this->_lastEpoc + i) : getTime(this->_lastEpoc + i)),
    (unsigned long)(this->_lastEpoc + i - getEpochTime()),
    this->_updateInterval / 1000, 
    (float)this->_updateInterval / MS_IN_H, 
    this->_updateDivisor);
  return buf;
}


// Parse our date/time into a structure, which we can then use elsewhere.
time_data NTPClient::parse_date_time(void) {
  return this->parse_date_time(this->getEpochTime());  
}
time_data NTPClient::parse_date_time(const double entryTime) {
  // Get epoch-time
  unsigned long outTime  = (unsigned long)entryTime;
  unsigned long walkTime;
  int dst_hour = 0;
  int tz_applied = 0;

  do {
    // Get basics
    _data.Wday   = (((outTime / 86400UL) + 4 ) % 7);
    _data.Hour   =  ((outTime % 86400UL) / 3600);
    _data.Minute =  ((outTime % 3600) / 60);
    _data.Second =   (outTime % 60);
    _data.MSec = int( (entryTime - (unsigned long)entryTime) * 10000 );

    // Now set walkTime to be in-days.
    walkTime = outTime / 86400UL;

    // Setup.
    unsigned long days = 0, year = 1970;
    uint8_t month;
    static const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};

    // Walk forward until we've found the number of years.
    while ((days += (LEAP_YEAR(year) ? 366 : 365)) <= walkTime) year++;

    // now it is days in this year, starting at 0
    walkTime -= days - (LEAP_YEAR(year) ? 366 : 365);
    days = 0;

    // Count forward until we've run out of days.
    for (month=0; month < 12; month++) {
      uint8_t monthLength;
      if (month == 1) { // february
        monthLength = LEAP_YEAR(year) ? 29 : 28;
      } else {
        monthLength = monthDays[month];
      }
      if (walkTime < monthLength) break;
      walkTime -= monthLength;
    }

    _data.Month = month + 1;
    _data.Year = year;
    _data.Day  = walkTime + 1;

    #ifdef DEBUG_NTP_PARSE_DATE_TIME
    debug_log_ln("%04d-%02d-%02d %02d:%02d:%02d.%04d eT:%16.4f oT:%10d tz_applied:%1d eu_dst:%1d dst:%1d timeOffset:%05d",
      _data.Year, _data.Month, _data.Day, _data.Hour, _data.Minute, _data.Second, _data.MSec,
      entryTime, outTime, tz_applied, _eu_dst, dst_hour, (long)_timeOffset);
    #endif

    if (_timeOffset == 0 && !_eu_dst) break;
    
    if (_eu_dst && !tz_applied) {
      // EU DST
      if ((_data.Month < 3) || (_data.Month > 10)) {
        // Jan-Feb, Nov-Dec: Normal time
        // exit if
      } else if ((_data.Month > 3) && (_data.Month < 10)) {
        // Apr-Sep: DST
        dst_hour = 1;
      } else if (_data.Month == 3) {
        // March
        if ((_data.Day > 24) && ((!_data.Wday && _data.Hour) || (_data.Wday && (_data.Day - _data.Wday > 24)))) {
            // Last Sunday of March, 01:00 UTC and after: DST
            dst_hour = 1;
        }
      } else if (_data.Day < 25) {
        // October, all before 25th: DST
        dst_hour = 1;
      } else if (!((!_data.Wday && _data.Hour) || (_data.Wday && (_data.Day - _data.Wday > 24)))) {
        // October 25th and after, but before last Sunday 01:00 UTC: DST
        dst_hour = 1;
      } // else normal time.
    }

    outTime = entryTime + (long)_timeOffset + (unsigned long)dst_hour * 3600UL;
    tz_applied++;
  } while (tz_applied < 2);

  _data.IsDST = dst_hour;

  return(_data);
}

int NTPClient::isDST(void) {
  if (!this->_eu_dst) return -1;
  return this->isDST(this->getEpochTime());
}
int NTPClient::isDST(const double epochtime) {
  if (!this->_eu_dst) return -1;
  this->parse_date_time(epochtime);
  return this->_data.IsDST;
}
char *NTPClient::getDateTimeISO(void) {
  return this->getDateTimeISO(this->getEpochTime());
}
char *NTPClient::getDateTimeISO(const double epochtime) {
  static char buf[MAX_ISO_TS];
  this->getDateTimeISO(buf, epochtime);
  return buf;
}
void NTPClient::getDateTimeISO(char *s) {
  this->getDateTimeISO(s, this->getEpochTime());
}
void NTPClient::getDateTimeISO(char *s, const double epochtime) {
  this->parse_date_time(epochtime);
  sprintf(s, "%04d-%02d-%02d %02d:%02d:%02d.%03d" , _data.Year, _data.Month, _data.Day, _data.Hour, _data.Minute, _data.Second, _data.MSec);
}
char *NTPClient::getTime(void) {
  return this->getTime(this->getEpochTime());
}
char *NTPClient::getTime(const double epochtime) {
  static char buf[MAX_ISO_TS];
  this->getTime(buf, epochtime);
  return buf;
}
void NTPClient::getTime(char *s) {
  this->getTime(s, this->getEpochTime());
}
void NTPClient::getTime(char *s, const double epochtime) {
  this->parse_date_time(epochtime);
  sprintf(s, "%02d:%02d:%02d.%03d" , _data.Hour, _data.Minute, _data.Second, _data.MSec);
}

void NTPClient::setNtpServer(const char *server) {
  this->setNtpServer(server, NTP_DEFAULT_LOCAL_PORT);
}
void NTPClient::setNtpServer(const char *server, int port) {
  if(strcmp(server, this->_poolServerName)) {
    this->_poolServerName = server;
    if(this->_udpSetup) {
      this->end();
      this->begin(port);
    }
  }
}

unsigned long NTPClient::getUpdateInterval(void) {
  return this->_updateInterval;
}
unsigned int NTPClient::getUpdateDivisor(void) {
  return this->_updateDivisor;
}
unsigned long NTPClient::getUpdateCount(void) {
  return this->_updateCount;
}
unsigned long NTPClient::getErrorCount(void) {
  return this->_errorCount;
}
unsigned long NTPClient::getDriftSampleCount(void) {
  return this->_driftSamples;
}

void NTPClient::setTimeOffset(const float timeOffset) {
  this->_timeOffset = timeOffset;
}
void NTPClient::setEUDST(const int eu_dst) {
  this->_eu_dst = eu_dst;
}
void NTPClient::setUpdateInterval(const unsigned long updateInterval) {
  this->_updateInterval = updateInterval;
}
void NTPClient::setDebugPackets(bool state) {
  this->_debugPackets = state;
}
void NTPClient::setDebugDrift(bool state) {
  this->_debugDrift = state;
}

void NTPClient::flushUDPPackets(void) {
  char t[128];
  int c = this->_udp->parsePacket();
  if (c && this->_debugPackets) debug_log_ln("Discarding %u stale bytes from UDP rx queue", c); 
  while (c > 0) c -= this->_udp->read((byte *)t, sizeof(t));
}

uint32_t NTPClient::jk_htonl(const uint32_t n) { return this->jk_ntohl(n); }
uint32_t NTPClient::jk_ntohl(const uint32_t n) {
  return
    ((n & 0xff)        << 24) |
    ((n & 0xff00)      <<  8) |
    ((n & 0xff0000U)   >>  8) |
    ((n & 0xff000000U) >> 24);
}

uint64_t NTPClient::jk_htonll(const uint64_t n) { return this->jk_ntohll(n); }
uint64_t NTPClient::jk_ntohll(const uint64_t n) {
  return 
    ((n << 56) |
    ((n << 40) & 0xff000000000000U) |
    ((n << 24) & 0xff0000000000U) |
    ((n <<  8) & 0xff00000000U) |
    ((n >>  8) & 0xff000000U) |
    ((n >> 24) & 0xff0000U) |
    ((n >> 40) & 0xff00U) |
    ( n >> 56));
}

// EOF
