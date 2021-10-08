/**
 * Author: sascha_lammers@gmx.de
 */

// provides access to I/O pins, digital and analog

#include <Arduino.h>
#include <wiring_private.h>

#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega328PB__)
#    error Invalid MCU
#endif

#if defined(__AVR_ATmega328PB__)
#    define is328PB(a, b) a
#else
#    define is328PB(a, b) b
#endif

namespace SFR {

    template<uint8_t _Pin>
    struct Pin {

        static constexpr volatile uint8_t &PORT()
        {
            static_assert(PORT_IO_ADDR() != 0, "N/A");
            return _SFR_IO8(PORT_IO_ADDR());
        }

        static constexpr uint8_t PORT_IO_ADDR()
        {
            return _Pin < 8 ? 0x0b :
                _Pin < 14 ? 0x05 :
                    _Pin < 20 ? 0x08 :
                        _Pin < 24 ? is328PB(0x03, 0) :
                            0;
        }

        static constexpr volatile uint8_t &PIN()
        {
            static_assert(PIN_IO_ADDR() != 0, "N/A");
            return _SFR_IO8(PIN_IO_ADDR());
        }

        static constexpr uint8_t PIN_IO_ADDR()
        {
            return _Pin < 8 ? 0x09 :
                _Pin < 14 ? 0x03 :
                    _Pin < 20 ? 0x06 :
                        _Pin < 24 ? is328PB(0x0c, 0) :
                            0;
        }

        static constexpr volatile uint8_t &DDR()
        {
            static_assert(DDR_IO_ADDR() != 0, "N/A");
            return _SFR_IO8(DDR_IO_ADDR());
        }

        static constexpr uint8_t DDR_IO_ADDR()
        {
            return _Pin < 8 ? 0x0a :
                _Pin < 14 ? 0x04 :
                    _Pin < 20 ? 0x07 :
                        _Pin < 24 ? is328PB(0x02, 0) :
                            0;
        }

        static constexpr volatile uint8_t &PCMSK()
        {
            return *digitalPinToPCMSK(_Pin);
        }

        static constexpr uint8_t PINmask()
        {
            static_assert(_PINbit() != 0xff, "N/A");
            return _BV(_PINbit());
        }

        static constexpr uint8_t PINbit()
        {
            static_assert(_PINbit() != 0xff, "N/A");
            return _PINbit();
        }

        static constexpr uint8_t _PINbit()
        {
            return _Pin < 8 ? _Pin :
                _Pin < 14 ? (_Pin - 8) :
                    _Pin < 20 ? (_Pin - 14) :
                        _Pin < 24 ? is328PB(_Pin - 20, 0) :
                            0xff;
        }

        static constexpr uint8_t PCICRbit()
        {
            return digitalPinToPCICRbit(_Pin);
        }

        static constexpr uint8_t TIMER()
        {
            static_assert(_TIMER() != 0, "N/A");
            return _TIMER();
        }

        static constexpr uint8_t _TIMER()
        {
            return _Pin == 0 ? is328PB(TIMER3A, 0) :
                _Pin == 1 ? is328PB(TIMER4A, 0) :
                    _Pin == 2 ? is328PB(TIMER4B, 0) :
                        _Pin == 3 ? TIMER2B :
                            _Pin == 5 ? TIMER0B :
                                _Pin == 6 ? TIMER0A :
                                    _Pin == 9 ? TIMER1A :
                                        _Pin == 10 ? TIMER1B :
                                            _Pin == 11 ? TIMER2A :
                                                0;
        }

        static constexpr volatile uint8_t &TCCR()
        {
            static_assert(TCCR_IO_ADDR() != 0, "N/A");
            return _SFR_IO8(TCCR_IO_ADDR());
        }

        static constexpr uint8_t TCCR_IO_ADDR()
        {
            return _Pin == 0 ? is328PB(0x8b - __SFR_OFFSET, 0) :
                _Pin == 1 ? is328PB(0xa0 - __SFR_OFFSET, 0) :
                    _Pin == 2 ? is328PB(0xa0 - __SFR_OFFSET, 0) :
                        _Pin == 3 ? 0xb0 :
                            _Pin == 5 ? 0x24 :
                                _Pin == 6 ? 0x24 :
                                    _Pin == 9 ? (0x80 - __SFR_OFFSET) :
                                        _Pin == 10 ? (0x80 - __SFR_OFFSET) :
                                            _Pin == 11 ? (0xb0 - __SFR_OFFSET) :
                                                0;
        }

        static constexpr uint8_t TCCR_MEM_ADDR()
        {
            return _Pin == 0 ? is328PB(0x8b, 0) :
                _Pin == 1 ? is328PB(0xa0, 0) :
                    _Pin == 2 ? is328PB(0xa0, 0) :
                        _Pin == 9 ? 0x80 :
                            _Pin == 10 ? 0x80 :
                                _Pin == 11 ? 0xb0 :
                                    0;
        }

        static constexpr uint8_t TCCRbit()
        {
            static_assert(_TCCRbit() != 0xff, "N/A");
            return _TCCRbit();
        }

        static constexpr uint8_t _TCCRbit()
        {
            return _Pin == 0 ? is328PB(COM3A1, 0xff) :
                _Pin == 1 ? is328PB(COM4A1, 0xff) :
                    _Pin == 2 ? is328PB(COM4B1, 0xff) :
                        _Pin == 3 ? COM2B1 :
                            _Pin == 5 ? COM0B1 :
                                _Pin == 6 ? COM0A1 :
                                    _Pin == 9 ? COM1A1 :
                                        _Pin == 10 ? COM1B1 :
                                            _Pin == 11 ? COM2A1 :
                                                0xff;
        }


        static constexpr volatile uint8_t &OCR()
        {
            static_assert(OCR_IO8_ADDR() != 0, "N/A");
            return _SFR_IO8(OCR_IO8_ADDR());
        }

        static constexpr uint8_t OCR_IO8_ADDR()
        {
            return _Pin == 0 ? is328PB(0x98, 0) :
                _Pin == 1 ? is328PB(0xa8, 0) :
                    _Pin == 2 ? is328PB(0xa8, 0) :
                        _Pin == 3 ? 0xb4 - __SFR_OFFSET :
                            _Pin == 5 ? 0x28 :
                                _Pin == 6 ? 0x27 :
                                    _Pin == 9 ? 0x88 - __SFR_OFFSET :
                                        _Pin == 10 ? 0x8a - __SFR_OFFSET :
                                            _Pin == 11 ? 0xb3 - __SFR_OFFSET :
                                                0;
        }

    };

}
