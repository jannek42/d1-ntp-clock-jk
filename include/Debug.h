#ifndef Debug_h
#define Debug_h

// If defined, output debug messages by default
#define DEBUG 1
#define MAX_DEBUG_MESSAGE 1536

// Output to Serial as-is
#define DLOG(...)         Debug->to_serial(__VA_ARGS__)
#define DEBUG_LOG(...)    Debug->to_serial(__VA_ARGS__)
#define debug_log(...)    Debug->to_serial(__VA_ARGS__)

// Output to Serial with \r\n appended
#define DLOGLN(...)       Debug->to_serial_ln(__VA_ARGS__)
#define DEBUG_LOG_LN(...) Debug->to_serial_ln(__VA_ARGS__)
#define debug_log_ln(...) Debug->to_serial_ln(__VA_ARGS__)

//
// Record a debug-message only if DEBUG is defined.
//
class DebugClass {
  public:
    void to_serial(const char *format, ...);
    void to_serial_ln(const char *format, ...);
    bool is_enabled(void);
    void set(bool state);
    void enable(void);
    void disable(void);
		
	private:
	  char _buffer[MAX_DEBUG_MESSAGE] = "";
    #ifdef DEBUG
    bool _enabled = true;
    #else
    bool _enabled = false;
    #endif
};

extern DebugClass *Debug;

#endif
