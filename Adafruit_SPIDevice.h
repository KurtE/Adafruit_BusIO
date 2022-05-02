#ifndef Adafruit_SPIDevice_h
#define Adafruit_SPIDevice_h

#include <Arduino.h>

#if !defined(SPI_INTERFACES_COUNT) ||                                          \
    (defined(SPI_INTERFACES_COUNT) && (SPI_INTERFACES_COUNT > 0))

#include <SPI.h>

// some modern SPI definitions don't have BitOrder enum
#if (defined(__AVR__) && !defined(ARDUINO_ARCH_MEGAAVR)) ||                    \
    defined(ESP8266) || defined(TEENSYDUINO) || defined(SPARK) ||              \
    defined(ARDUINO_ARCH_SPRESENSE) || defined(MEGATINYCORE) ||                \
    defined(DXCORE) || defined(ARDUINO_AVR_ATmega4809) ||                      \
    defined(ARDUINO_AVR_ATmega4808) || defined(ARDUINO_AVR_ATmega3209) ||      \
    defined(ARDUINO_AVR_ATmega3208) || defined(ARDUINO_AVR_ATmega1609) ||      \
    defined(ARDUINO_AVR_ATmega1608) || defined(ARDUINO_AVR_ATmega809) ||       \
    defined(ARDUINO_AVR_ATmega808) || defined(ARDUINO_ARCH_ARC32)

typedef enum _BitOrder {
  SPI_BITORDER_MSBFIRST = MSBFIRST,
  SPI_BITORDER_LSBFIRST = LSBFIRST,
} BusIOBitOrder;

#elif defined(ESP32) || defined(__ASR6501__) || defined(__ASR6502__)

// some modern SPI definitions don't have BitOrder enum and have different SPI
// mode defines
typedef enum _BitOrder {
  SPI_BITORDER_MSBFIRST = SPI_MSBFIRST,
  SPI_BITORDER_LSBFIRST = SPI_LSBFIRST,
} BusIOBitOrder;

#else
// Some platforms have a BitOrder enum but its named MSBFIRST/LSBFIRST
#define SPI_BITORDER_MSBFIRST MSBFIRST
#define SPI_BITORDER_LSBFIRST LSBFIRST
typedef BitOrder BusIOBitOrder;
#endif

#if defined(__IMXRT1062__) // Teensy 4.x
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif defined(__AVR__) || defined(TEENSYDUINO)
typedef volatile uint8_t BusIO_PortReg;
typedef uint8_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif defined(ESP8266) || defined(ESP32) || defined(__SAM3X8E__) ||            \
    defined(ARDUINO_ARCH_SAMD)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#define BUSIO_USE_FAST_PINIO

#elif (defined(__arm__) || defined(ARDUINO_FEATHER52)) &&                      \
    !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_RP2040)
typedef volatile uint32_t BusIO_PortReg;
typedef uint32_t BusIO_PortMask;
#if !defined(__ASR6501__) && !defined(__ASR6502__)
#define BUSIO_USE_FAST_PINIO
#endif

#else
#undef BUSIO_USE_FAST_PINIO
#endif

/**! The class which defines how we will talk to this device over SPI **/
class Adafruit_SPIDevice {
public:
  Adafruit_SPIDevice(int8_t cspin, uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0, SPIClass *theSPI = &SPI);

  Adafruit_SPIDevice(int8_t cspin, int8_t sck, int8_t miso, int8_t mosi,
                     uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0);
  ~Adafruit_SPIDevice();

  bool begin(void);
  bool read(uint8_t *buffer, size_t len, uint8_t sendvalue = 0xFF);
  bool write(const uint8_t *buffer, size_t len,
             const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
  bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       uint8_t sendvalue = 0xFF);
  bool write_and_read(uint8_t *buffer, size_t len);

  uint8_t transfer(uint8_t send);
  void transfer(uint8_t *buffer, size_t len);
  void beginTransaction(void);
  void endTransaction(void);

private:
  SPIClass *_spi;
  SPISettings *_spiSetting;
  uint32_t _freq;
  BusIOBitOrder _dataOrder;
  uint8_t _dataMode;
  void setChipSelect(int value);

  int8_t _cs, _sck, _mosi, _miso;
#ifdef BUSIO_USE_FAST_PINIO
  BusIO_PortReg *mosiPort, *clkPort, *misoPort, *csPort;
  BusIO_PortMask mosiPinMask, misoPinMask, clkPinMask, csPinMask;

#if defined(__IMXRT1062__) // Teensy 4.x
  //#define portOutputRegister(pin)  ((digital_pin_to_info_PGM[(pin)].reg + 0))
  //#define portSetRegister(pin)     ((digital_pin_to_info_PGM[(pin)].reg + 33))
  //#define portClearRegister(pin)   ((digital_pin_to_info_PGM[(pin)].reg + 34))
  //#define portToggleRegister(pin)  ((digital_pin_to_info_PGM[(pin)].reg + 35))

  inline void setMosi(bool towrite) {
    if (towrite)
      *(mosiPort + 33) = mosiPinMask;
    else
      *(mosiPort + 34) = mosiPinMask;
  }
  inline void setClkHigh() { *(clkPort + 33) = clkPinMask; }
  inline void setClkLow() { *(clkPort + 34) = clkPinMask; }
  inline void setCSHigh() { *(csPort + 33) = csPinMask; }
  inline void setCSLow() { *(csPort + 34) = csPinMask; }
  inline bool getMisoState() { return (*misoPort & misoPinMask); }
#else
  inline void setMosi(bool towrite) {
    if (towrite)
      *mosiPort |= mosiPinMask;
    else
      *mosiPort &= ~mosiPinMask;
  }
  inline void setClkHigh() { *clkPort |= clkPinMask; }
  inline void setClkLow() { *clkPort &= ~clkPinMask; }
  inline void setCSHigh() { *csPort |= csPinMask; }
  inline void setCSLow() { *csPort &= ~csPinMask; }
  inline bool getMisoState() { return (*misoPort & misoPinMask); }
#endif
#else
  inline void setMosi(bool towrite) { digitalWrite(_mosi, towrite); }
  inline void setClkHigh() { digitalWrite(_sck, HIGH); }
  inline void setClkLow() { digitalWrite(_sck, LOW); }
  inline void setCSHigh() { digitalWrite(_cs, HIGH); }
  inline void setCSLow() { digitalWrite(_cs, LOW); }
  inline bool getMisoState() { return (digitalRead(_miso)); }
#endif
  bool _begun;
};

#endif // has SPI defined
#endif // Adafruit_SPIDevice_h
