#ifndef PLATFORM_BRIDGE_H
#define PLATFORM_BRIDGE_H

#if defined(TARGET_TEENSY41)
#include <Arduino.h>
#include <SD.h>
#include "ODriveCAN.h"
#include "FlexCAN_T4.h"

#if defined(__arm__) && defined(TEENSYDUINO)
#if defined(__has_include) && __has_include(<EventResponder.h>)
#define SPI_HAS_TRANSFER_ASYNC 1
#include <DMAChannel.h>
#include <EventResponder.h>
#endif
#endif

#elif defined(TARGET_NATIVE)
#include <iostream>
#include <chrono>
#include <thread>
#include <istream>
#include <fstream>
#include <filesystem>
#include <cstring>
#include <array>
//#include <format>
#include <stdint.h>
#include "PlatformBridgeIMXRT.h"
#include "PlatformBridgeKinetis.h"

//#include "csv.h"
#endif

#if defined(TARGET_TEENSY41) || defined(TARGET_NATIVE)
#include <vector>
#include <string>
#include <functional>
#endif

/*
 * PlatformBridge.h
 *
 * Created on: 2025-06-30 by Ethan Chen
 * Maintained by Ethan Chen
 * Description: This file acts as an abstraction layer for the portions of the API used across multiple platforms
 */

// add csv and implement for tadpole sensors
// Make everything use CStrings if possible, but otherwise compensate with std::string derived class (yes ik it's bad practice)
// Move to multithreading (complete data no pausing unless explicitly required otherwise use fbo like buffer switching system and deprioritizing recentness of data)

#if defined(TARGET_TEENSY41)

// Not required, but list the portions of the APIs used here so we can keep track of things and move it to another namespace if needed
using ::File;
using ::Serial;
using ::delay;
using ::SD;
using ::String;

#elif defined(TARGET_NATIVE)

#define HIGH 0x0
#define LOW 0x1

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

struct Pin {
	uint8_t mode;
	bool pullup;
};
struct DigitalPin : Pin {
	uint8_t value;
};
extern std::array<DigitalPin, UINT8_MAX> digital_pins;
struct AnalogPin : Pin {
	int value;
};
extern std::array<AnalogPin, UINT8_MAX> analog_pins;

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);
void analogWrite(uint8_t pin, int val);

typedef bool boolean;

class String : public std::string {
public:
	String();
	String(const std::string& str);
	String(const char* s);
	String(const std::string& str, size_t pos, size_t len = std::string::npos);

	void trim();
};

extern std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
void delay(unsigned long ms);
void delayMicroseconds(unsigned long ms);
unsigned long millis();
unsigned long micros();

class usb_serial_class {
public:
	usb_serial_class();

	void begin(long bitsPerSecond);
	void setTimeout(long msTime);
	template <typename T>
	size_t print(T in);
	template <typename T>
	size_t println(T in);
	size_t write(const char* msg, size_t len);
	void flush();
	size_t readBytes(char* buffer, size_t length);
	String readString(size_t length = 120);
	size_t readBytesUntil(char terminator, char* buffer, size_t length);
	String readStringUntil(char terminator, size_t length = 120);
};

extern usb_serial_class Serial;

#define FILE_READ 0
#define FILE_WRITE 1

class File {
public:
	std::string name_str;
	std::filesystem::path path;
	char mode;
	std::fstream stream;
	std::filesystem::directory_iterator dir_cursor;
	inline const static std::filesystem::directory_iterator dir_end;

	File();
	File(const char* path_str, char mode = FILE_WRITE);

	operator bool() const;
	File openNextFile(uint8_t mode = 0);
	const char* name();
	void close();
	bool available();
	template <typename T>
	size_t print(T in);
	template <typename T>
	size_t println(T in);
	void flush();
	size_t readBytes(char* buffer, size_t length);
	String readString(size_t length = 120);
	size_t readBytesUntil(char terminator, char* buffer, size_t length, std::istream& stream = std::cin);
	String readStringUntil(char terminator, size_t length = 120);
};

class SDClass {
public:
	SDClass();

	bool begin(uint8_t csPin = 0);
	File open(const char* path_str, char mode = FILE_WRITE);
	bool exists(const char* file_path_str);
	bool remove(const char* file_path_str);
};

extern SDClass SD;


typedef enum FLEXCAN_RXTX { // https://github.com/tonton81/FlexCAN_T4/blob/master/FlexCAN_T4.h
  TX,
  RX,
  LISTEN_ONLY
} FLEXCAN_RXTX;

typedef struct CAN_message_t {
  uint32_t id = 0;          // can identifier
  uint16_t timestamp = 0;   // FlexCAN time when message arrived
  uint8_t idhit = 0; // filter that id came from
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;  // remote transmission request packet type
    bool overrun = 0; // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = { 0 };       // data
  int8_t mb = 0;       // used to identify mailbox reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  bool seq = 0;         // sequential frames
} CAN_message_t;

typedef struct CANFD_message_t {
  uint32_t id = 0;          // can identifier
  uint16_t timestamp = 0;   // FlexCAN time when message arrived
  uint8_t idhit = 0; // filter that id came from
  bool brs = 1;        // baud rate switching for data
  bool esi = 0;        // error status indicator
  bool edl = 1;        // extended data length (for RX, 0 == CAN2.0, 1 == FD)
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool overrun = 0; // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[64] = { 0 };       // data
  int8_t mb = 0;       // used to identify mailbox reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  bool seq = 0;         // sequential frames
} CANFD_message_t;

class FlexCAN_T4_Base {
  public:
    virtual void flexcan_interrupt() = 0;
    virtual void setBaudRate(uint32_t baud = 1000000, FLEXCAN_RXTX listen_only = TX) = 0;
    virtual uint64_t events() = 0;
    virtual int write(const CANFD_message_t &msg) = 0;
    virtual int write(const CAN_message_t &msg) = 0;
    virtual bool isFD() = 0;
    virtual uint8_t getFirstTxBoxSize() = 0;
};


class DMABaseClass { // https://github.com/PaulStoffregen/cores/blob/f6ca1f1d61058bf1a544757d25a81954e24a43fd/teensy3/DMAChannel.h#L72
public:
	typedef struct __attribute__((packed, aligned(4))) {
		volatile const void * volatile SADDR;
		int16_t SOFF;
		union { uint16_t ATTR;
			struct { uint8_t ATTR_DST; uint8_t ATTR_SRC; }; };
		union { uint32_t NBYTES; uint32_t NBYTES_MLNO;
			uint32_t NBYTES_MLOFFNO; uint32_t NBYTES_MLOFFYES; };
		int32_t SLAST;
		volatile void * volatile DADDR;
		int16_t DOFF;
		union { volatile uint16_t CITER;
			volatile uint16_t CITER_ELINKYES; volatile uint16_t CITER_ELINKNO; };
		int32_t DLASTSGA;
		volatile uint16_t CSR;
		union { volatile uint16_t BITER;
			volatile uint16_t BITER_ELINKYES; volatile uint16_t BITER_ELINKNO; };
	} TCD_t;
	TCD_t *TCD;

	/***************************************/
	/**    Data Transfer                  **/
	/***************************************/

	// Use a single variable as the data source.  Typically a register
	// for receiving data from one of the hardware peripherals is used.
	void source(volatile const signed char &p) { source(*(volatile const uint8_t *)&p); }
	void source(volatile const unsigned char &p) {
		TCD->SADDR = &p;
		TCD->SOFF = 0;
		TCD->ATTR_SRC = 0;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 1;
		TCD->SLAST = 0;
	}
	void source(volatile const signed short &p) { source(*(volatile const uint16_t *)&p); }
	void source(volatile const unsigned short &p) {
		TCD->SADDR = &p;
		TCD->SOFF = 0;
		TCD->ATTR_SRC = 1;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 2;
		TCD->SLAST = 0;
	}
	void source(volatile const signed int &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const unsigned int &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const signed long &p) { source(*(volatile const uint32_t *)&p); }
	void source(volatile const unsigned long &p) {
		TCD->SADDR = &p;
		TCD->SOFF = 0;
		TCD->ATTR_SRC = 2;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 4;
		TCD->SLAST = 0;
	}

	// Use a buffer (array of data) as the data source.  Typically a
	// buffer for transmitting data is used.
	void sourceBuffer(volatile const signed char p[], unsigned int len) {
		sourceBuffer((volatile const uint8_t *)p, len); }
	void sourceBuffer(volatile const unsigned char p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 1;
		TCD->ATTR_SRC = 0;
		TCD->NBYTES = 1;
		TCD->SLAST = -len;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void sourceBuffer(volatile const signed short p[], unsigned int len) {
		sourceBuffer((volatile const uint16_t *)p, len); }
	void sourceBuffer(volatile const unsigned short p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 2;
		TCD->ATTR_SRC = 1;
		TCD->NBYTES = 2;
		TCD->SLAST = -len;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void sourceBuffer(volatile const signed int p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const unsigned int p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const signed long p[], unsigned int len) {
		sourceBuffer((volatile const uint32_t *)p, len); }
	void sourceBuffer(volatile const unsigned long p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 4;
		TCD->ATTR_SRC = 2;
		TCD->NBYTES = 4;
		TCD->SLAST = -len;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	// Use a circular buffer as the data source
	void sourceCircular(volatile const signed char p[], unsigned int len) {
		sourceCircular((volatile const uint8_t *)p, len); }
	void sourceCircular(volatile const unsigned char p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 1;
		TCD->ATTR_SRC = ((31 - __builtin_clz(len)) << 3);
		TCD->NBYTES = 1;
		TCD->SLAST = 0;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void sourceCircular(volatile const signed short p[], unsigned int len) {
		sourceCircular((volatile const uint16_t *)p, len); }
	void sourceCircular(volatile const unsigned short p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 2;
		TCD->ATTR_SRC = ((31 - __builtin_clz(len)) << 3) | 1;
		TCD->NBYTES = 2;
		TCD->SLAST = 0;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void sourceCircular(volatile const signed int p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const unsigned int p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const signed long p[], unsigned int len) {
		sourceCircular((volatile const uint32_t *)p, len); }
	void sourceCircular(volatile const unsigned long p[], unsigned int len) {
		TCD->SADDR = p;
		TCD->SOFF = 4;
		TCD->ATTR_SRC = ((31 - __builtin_clz(len)) << 3) | 2;
		TCD->NBYTES = 4;
		TCD->SLAST = 0;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	// Use a single variable as the data destination.  Typically a register
	// for transmitting data to one of the hardware peripherals is used.
	void destination(volatile signed char &p) { destination(*(volatile uint8_t *)&p); }
	void destination(volatile unsigned char &p) {
		TCD->DADDR = &p;
		TCD->DOFF = 0;
		TCD->ATTR_DST = 0;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 1;
		TCD->DLASTSGA = 0;
	}
	void destination(volatile signed short &p) { destination(*(volatile uint16_t *)&p); }
	void destination(volatile unsigned short &p) {
		TCD->DADDR = &p;
		TCD->DOFF = 0;
		TCD->ATTR_DST = 1;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 2;
		TCD->DLASTSGA = 0;
	}
	void destination(volatile signed int &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile unsigned int &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile signed long &p) { destination(*(volatile uint32_t *)&p); }
	void destination(volatile unsigned long &p) {
		TCD->DADDR = &p;
		TCD->DOFF = 0;
		TCD->ATTR_DST = 2;
		if ((uint32_t)&p < 0x40000000 || TCD->NBYTES == 0) TCD->NBYTES = 4;
		TCD->DLASTSGA = 0;
	}

	// Use a buffer (array of data) as the data destination.  Typically a
	// buffer for receiving data is used.
	void destinationBuffer(volatile signed char p[], unsigned int len) {
		destinationBuffer((volatile uint8_t *)p, len); }
	void destinationBuffer(volatile unsigned char p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 1;
		TCD->ATTR_DST = 0;
		TCD->NBYTES = 1;
		TCD->DLASTSGA = -len;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void destinationBuffer(volatile signed short p[], unsigned int len) {
		destinationBuffer((volatile uint16_t *)p, len); }
	void destinationBuffer(volatile unsigned short p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 2;
		TCD->ATTR_DST = 1;
		TCD->NBYTES = 2;
		TCD->DLASTSGA = -len;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void destinationBuffer(volatile signed int p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile unsigned int p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile signed long p[], unsigned int len) {
		destinationBuffer((volatile uint32_t *)p, len); }
	void destinationBuffer(volatile unsigned long p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 4;
		TCD->ATTR_DST = 2;
		TCD->NBYTES = 4;
		TCD->DLASTSGA = -len;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	// Use a circular buffer as the data destination
	void destinationCircular(volatile signed char p[], unsigned int len) {
		destinationCircular((volatile uint8_t *)p, len); }
	void destinationCircular(volatile unsigned char p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 1;
		TCD->ATTR_DST = ((31 - __builtin_clz(len)) << 3);
		TCD->NBYTES = 1;
		TCD->DLASTSGA = 0;
		TCD->BITER = len;
		TCD->CITER = len;
	}
	void destinationCircular(volatile signed short p[], unsigned int len) {
		destinationCircular((volatile uint16_t *)p, len); }
	void destinationCircular(volatile unsigned short p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 2;
		TCD->ATTR_DST = ((31 - __builtin_clz(len)) << 3) | 1;
		TCD->NBYTES = 2;
		TCD->DLASTSGA = 0;
		TCD->BITER = len / 2;
		TCD->CITER = len / 2;
	}
	void destinationCircular(volatile signed int p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile unsigned int p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile signed long p[], unsigned int len) {
		destinationCircular((volatile uint32_t *)p, len); }
	void destinationCircular(volatile unsigned long p[], unsigned int len) {
		TCD->DADDR = p;
		TCD->DOFF = 4;
		TCD->ATTR_DST = ((31 - __builtin_clz(len)) << 3) | 2;
		TCD->NBYTES = 4;
		TCD->DLASTSGA = 0;
		TCD->BITER = len / 4;
		TCD->CITER = len / 4;
	}

	/*************************************************/
	/**    Quantity of Data to Transfer             **/
	/*************************************************/

	// Set the data size used for each triggered transfer
	void transferSize(unsigned int len) {
		if (len == 16) {
			TCD->NBYTES = 16;
			if (TCD->SOFF != 0) TCD->SOFF = 16;
			if (TCD->DOFF != 0) TCD->DOFF = 16;
			TCD->ATTR = (TCD->ATTR & 0xF8F8) | 0x0404;
		} else if (len == 4) {
			TCD->NBYTES = 4;
			if (TCD->SOFF != 0) TCD->SOFF = 4;
			if (TCD->DOFF != 0) TCD->DOFF = 4;
			TCD->ATTR = (TCD->ATTR & 0xF8F8) | 0x0202;
		} else if (len == 2) {
			TCD->NBYTES = 2;
			if (TCD->SOFF != 0) TCD->SOFF = 2;
			if (TCD->DOFF != 0) TCD->DOFF = 2;
			TCD->ATTR = (TCD->ATTR & 0xF8F8) | 0x0101;
		} else {
			TCD->NBYTES = 1;
			if (TCD->SOFF != 0) TCD->SOFF = 1;
			if (TCD->DOFF != 0) TCD->DOFF = 1;
			TCD->ATTR = TCD->ATTR & 0xF8F8;
		}
	}

	// Set the number of transfers (number of triggers until complete)
	void transferCount(unsigned int len) {
		if (!(TCD->BITER & DMA_TCD_BITER_ELINK)) {
			if (len > 32767) return;
			TCD->BITER = len;
			TCD->CITER = len;
		} else {
			if (len > 511) return;
			TCD->BITER = (TCD->BITER & 0xFE00) | len;
			TCD->CITER = (TCD->CITER & 0xFE00) | len;
		}
	}

	/*************************************************/
	/**    Special Options / Features               **/
	/*************************************************/

	void interruptAtCompletion(void) {
		TCD->CSR |= DMA_TCD_CSR_INTMAJOR;
	}

	void interruptAtHalf(void) {
		TCD->CSR |= DMA_TCD_CSR_INTHALF;
	}

	void disableOnCompletion(void) {
		TCD->CSR |= DMA_TCD_CSR_DREQ;
	}

	void replaceSettingsOnCompletion(const DMABaseClass &settings) {
		TCD->DLASTSGA = (int32_t)(settings.TCD);
		TCD->CSR &= ~DMA_TCD_CSR_DONE;
		TCD->CSR |= DMA_TCD_CSR_ESG;
	}

protected:
	// users should not be able to create instances of DMABaseClass, which
	// require the inheriting class to initialize the TCD pointer.
	DMABaseClass() {}

	static inline void copy_tcd(TCD_t *dst, const TCD_t *src) {
		dst->CSR = 0;
		const uint32_t *p = (const uint32_t *)src;
		uint32_t *q = (uint32_t *)dst;
		uint32_t t1, t2, t3, t4;
		t1 = *p++; t2 = *p++; t3 = *p++; t4 = *p++;
		*q++ = t1; *q++ = t2; *q++ = t3; *q++ = t4;
		t1 = *p++; t2 = *p++; t3 = *p++; t4 = *p++;
		*q++ = t1; *q++ = t2; *q++ = t3; *q++ = t4;
	}
};


// DMASetting represents settings stored only in memory, which can be
// applied to any DMA channel.

class DMASetting : public DMABaseClass {
public:
	DMASetting() {
		TCD = &tcddata;
	}
	DMASetting(const DMASetting &c) {
		TCD = &tcddata;
		*this = c;
	}
	DMASetting(const DMABaseClass &c) {
		TCD = &tcddata;
		*this = c;
	}
	DMASetting & operator = (const DMABaseClass &rhs) {
		copy_tcd(TCD, rhs.TCD);
		return *this;
	}
private:
	TCD_t tcddata __attribute__((aligned(32)));
};

class DMAChannel : public DMABaseClass { // https://github.com/PaulStoffregen/cores/blob/f6ca1f1d61058bf1a544757d25a81954e24a43fd/teensy3/DMAChannel.h#L422
public:
	/*************************************************/
	/**    Channel Allocation                       **/
	/*************************************************/

	DMAChannel() {
		begin();
	}
	DMAChannel(const DMAChannel &c) {
		TCD = c.TCD;
		channel = c.channel;
	}
	DMAChannel(const DMASetting &c) {
		begin();
		copy_tcd(TCD, c.TCD);
	}
	DMAChannel(bool allocate) {
		if (allocate) begin();
	}
	DMAChannel & operator = (const DMAChannel &rhs) {
		if (channel != rhs.channel) {
			release();
			TCD = rhs.TCD;
			channel = rhs.channel;
		}
		return *this;
	}
	DMAChannel & operator = (const DMASetting &rhs) {
		copy_tcd(TCD, rhs.TCD);
		return *this;
	}
	~DMAChannel() {
		release();
	}
	void begin(bool force_initialization = false);
private:
	void release(void);

public:
	/***************************************/
	/**    Triggering                     **/
	/***************************************/

	// Triggers cause the DMA channel to actually move data.  Each
	// trigger moves a single data unit, which is typically 8, 16 or
	// 32 bits.  If a channel is configured for 200 transfers

	// Use a hardware trigger to make the DMA channel run
	void triggerAtHardwareEvent(uint8_t source) {
		volatile uint8_t *mux;
		mux = (volatile uint8_t *)&(DMAMUX0_CHCFG0) + channel;
		*mux = 0;
		*mux = (source & 63) | DMAMUX_ENABLE;
	}

	// Use another DMA channel as the trigger, causing this
	// channel to trigger after each transfer is makes, except
	// the its last transfer.  This effectively makes the 2
	// channels run in parallel until the last transfer
	void triggerAtTransfersOf(DMABaseClass &ch) {
		ch.TCD->BITER = (ch.TCD->BITER & ~DMA_TCD_BITER_ELINKYES_LINKCH_MASK)
		  | DMA_TCD_BITER_ELINKYES_LINKCH(channel) | DMA_TCD_BITER_ELINKYES_ELINK;
		ch.TCD->CITER = ch.TCD->BITER ;
	}

	// Use another DMA channel as the trigger, causing this
	// channel to trigger when the other channel completes.
	void triggerAtCompletionOf(DMABaseClass &ch) {
		ch.TCD->CSR = (ch.TCD->CSR & ~(DMA_TCD_CSR_MAJORLINKCH_MASK|DMA_TCD_CSR_DONE))
		  | DMA_TCD_CSR_MAJORLINKCH(channel) | DMA_TCD_CSR_MAJORELINK;
	}

	// Cause this DMA channel to be continuously triggered, so
	// it will move data as rapidly as possible, without waiting.
	// Normally this would be used with disableOnCompletion().
	void triggerContinuously(void) {
		volatile uint8_t *mux = (volatile uint8_t *)&DMAMUX0_CHCFG0;
		mux[channel] = 0;
#if DMAMUX_NUM_SOURCE_ALWAYS >= DMA_NUM_CHANNELS
		mux[channel] = DMAMUX_SOURCE_ALWAYS0 + channel;	
#else
		// search for an unused "always on" source
		unsigned int i = DMAMUX_SOURCE_ALWAYS0;
		for (i = DMAMUX_SOURCE_ALWAYS0;
		  i < DMAMUX_SOURCE_ALWAYS0 + DMAMUX_NUM_SOURCE_ALWAYS; i++) {
			unsigned int ch;
			for (ch=0; ch < DMA_NUM_CHANNELS; ch++) {
				if (mux[ch] == i) break;
			}
			if (ch >= DMA_NUM_CHANNELS) {
				mux[channel] = (i | DMAMUX_ENABLE);
				return;
			}
		}
#endif
	}

	// Manually trigger the DMA channel.
	void triggerManual(void) {
		DMA_SSRT = channel;
	}


	/***************************************/
	/**    Interrupts                     **/
	/***************************************/

	// An interrupt routine can be run when the DMA channel completes
	// the entire transfer, and also optionally when half of the
	// transfer is completed.
	void attachInterrupt(void (*isr)(void)) {
		_VectorsRam[channel + IRQ_DMA_CH0 + 16] = isr;
		NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + channel);
	}

	void attachInterrupt(void (*isr)(void), uint8_t prio) {
		_VectorsRam[channel + IRQ_DMA_CH0 + 16] = isr;
		NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + channel);
		NVIC_SET_PRIORITY(IRQ_DMA_CH0 + channel, prio);
	}
	
	void detachInterrupt(void) {
		NVIC_DISABLE_IRQ(IRQ_DMA_CH0 + channel);
	}

	void clearInterrupt(void) {
		DMA_CINT = channel;
	}


	/***************************************/
	/**    Enable / Disable               **/
	/***************************************/

	void enable(void) {
		DMA_SERQ = channel;
	}
	void disable(void) {
		DMA_CERQ = channel;
	}

	/***************************************/
	/**    Status                         **/
	/***************************************/

	bool complete(void) {
		if (TCD->CSR & DMA_TCD_CSR_DONE) return true;
		return false;
	}
	void clearComplete(void) {
		DMA_CDNE = channel;
	}
	bool error(void) {
		if (DMA_ERR & (1<<channel)) return true;
		return false;
	}
	void clearError(void) {
		DMA_CERR = channel;
	}
	void * sourceAddress(void) {
		return (void *)(TCD->SADDR);
	}
	void * destinationAddress(void) {
		return (void *)(TCD->DADDR);
	}

	/***************************************/
	/**    Direct Hardware Access         **/
	/***************************************/

	// For complex and unusual configurations not possible with the above
	// functions, the Transfer Control Descriptor (TCD) and channel number
	// can be used directly.  This leads to less portable and less readable
	// code, but direct control of all parameters is possible.
	uint8_t channel;
	// TCD is accessible due to inheritance from DMABaseClass
};

#endif

#endif

#include "PlatformBridge.tpp"