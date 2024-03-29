/**
 * Author: sascha_lammers@gmx.de
 */

#include "helpers.h"

#if 1

// saves 668 byte
// malloc is not used anywhere else

size_t Print::__printf(vsnprint_t func, const char *format, va_list arg)
{
    char buf[128];
    int len = func(buf, sizeof(buf), format, arg);
    write(buf, len);
    return len;
}

#else

size_t Print::__printf(vsnprint_t func, const char *format, va_list arg)
{
    char buf[64];
    char *temp = buf;
    int len = func(buf, sizeof(buf), format, arg);
    if (len >= (int)sizeof(buf) - 1) {
        temp = (char *)malloc(len + 2);
        if (!temp) {
            return 0;
        }
        len = func(temp, len, format, arg);
    }
    write(temp, len);
    if (temp != buf) {
        free(temp);
    }
    return len;
}

#endif

size_t Print::printf(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    size_t result = __printf( vsnprintf, format, arg);
    va_end(arg);
    return result;
}

size_t Print::printf_P(PGM_P format, ...)
{
    va_list arg;
    va_start(arg, format);
    size_t result = __printf(vsnprintf_P, format, arg);
    va_end(arg);
    return result;
}

#if DEBUG && 0

uint8_t _debug_level = 0;

void debug_print_millis()
{
    Serial.printf_P(PSTR("+REM=%05.5lu "), millis());
    Serial.flush();
}

void __debug_printf(const char *format, ...)
{
    debug_print_millis();
    va_list arg;
    va_start(arg, format);
    Serial.__printf(vsnprintf_P, format, arg);
    Serial.flush();
    va_end(arg);
}
// #define debug_printf(fmt, ...)              { debug_print_millis(); Serial.printf_P(PSTR(fmt), ##__VA_ARGS__); Serial.flush(); }

void __debug_print_memory(void *ptr, size_t size)
{
    debug_print_millis();
    auto data = (uint8_t *)ptr;
    uint8_t n = 0;
    Serial.printf_P(PSTR("[%p:%04x]"), ptr, size);
    while(size--) {
        Serial.printf_P(PSTR("%02x"), *data++);
        if ((++n % 2 == 0) && size > 1) {
            Serial.print(' ');
            Serial.flush();
        }
    }
    Serial.println();
    Serial.flush();
}

int assert_failed()
{
    Serial.print(F("\nASSERT FAILED\n"));
    Serial.flush();
    abort();
}

#endif

#if 0
/*
ATmega1280      1e9703
ATmega1281      1e9704
ATmega1284P     1e9705
ATmega1284      1e9706
ATmega128A      1e9702
ATmega128RFA1   1ea701
ATmega128       1e9702
ATmega162       1e9404
ATmega164A      1e940f
ATmega164PA     1e940a
ATmega164P      1e940a
ATmega165A      1e9410
ATmega165PA     1e9407
ATmega165P      1e9407
ATmega168A      1e9406
ATmega168PA     1e940b
ATmega168P      1e940b
ATmega168       1e9406
ATmega169A      1e9411
ATmega169PA     1e9405
ATmega169P      1e9405
ATmega16A       1e9403
ATmega16HVB     1e940d
ATmega16M1      1e9484
ATmega16U2      1e9489
ATmega16U4      1e9488
ATmega16        1e9403
ATmega2560      1e9801
ATmega2561      1e9802
ATmega324A      1e9515
ATmega324PA     1e9511
ATmega324P      1e9508
ATmega3250A     1e9506
ATmega3250PA    1e950e
ATmega3250P     1e950e
ATmega3250      1e9506
ATmega325A      1e9505
ATmega325PA     1e950d
ATmega325P      1e950d
ATmega325       1e9505
ATmega328P      1e950f
ATmega328       1e9514
ATmega3290A     1e9504
ATmega3290PA    1e950c
ATmega3290P     1e950c
ATmega3290      1e9504
ATmega329A      1e9503
ATmega329PA     1e950b
ATmega329P      1e950b
ATmega329       1e9503
ATmega32A       1e9502
ATmega32C1      1e9586
ATmega32HVB     1e9510
ATmega32M1      1e9584
ATmega32U2      1e958a
ATmega32U4      1e9587
ATmega32        1e9502
ATmega48A       1e9205
ATmega48PA      1e920a
ATmega48P       1e920a
ATmega48        1e9205
ATmega640       1e9608
ATmega644A      1e9609
ATmega644PA     1e960a
ATmega644P      1e960a
ATmega644       1e9609
ATmega6450A     1e9606
ATmega6450P     1e960e
ATmega6450      1e9606
ATmega645A      1e9605
ATmega645P      1e960D
ATmega645       1e9605
ATmega6490A     1e9604
ATmega6490P     1e960C
ATmega6490      1e9604
ATmega649A      1e9603
ATmega649P      1e960b
ATmega649       1e9603
ATmega64A       1e9602
ATmega64C1      1e9686
ATmega64M1      1e9684
ATmega64        1e9602
ATmega8515      1e9306
ATmega8535      1e9308
ATmega88A       1e930a
ATmega88PA      1e930f
ATmega88P       1e930f
ATmega88        1e930a
ATmega8A        1e9307
ATmega8U2       1e9389
ATmega8         1e9307
*/

uint8_t *get_signature(uint8_t *sig) {
    auto ptr = sig;
    *ptr++ = boot_signature_byte_get(0);
    *ptr++ = boot_signature_byte_get(2);
    *ptr++ = boot_signature_byte_get(4);
    return sig;
}

std::unique_ptr<uint8_t> get_mcu_type(char *&mcu, uint8_t *&sig, uint8_t *&fuses) {
    const size_t mcu_size = 17;
    auto buffer = new uint8_t[3 + 3 + mcu_size + 1];
    auto ptr = buffer;

    sig = ptr;
    ptr = get_signature(ptr) + 3;

    fuses = ptr;
    *ptr++ = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
    *ptr++ = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
    *ptr++ = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);

    mcu = reinterpret_cast<char *>(ptr);
    *mcu = 0;
    *(mcu + mcu_size) = 0;

    if (sig[0] == 0x1e) {
        if (sig[1] == 0x93) {
            switch(sig[2]) {
                case 0x0a:
                    strncpy_P(mcu, PSTR("ATmega88"), mcu_size);
                    break;
                case 0x0f:
                    strncpy_P(mcu, PSTR("ATmega88P"), mcu_size);
                    break;
            }
        }
        else if (sig[1] == 0x95) {
            switch(sig[2]) {
                case 0x02:
                    strncpy_P(mcu, PSTR("ATmega32"), mcu_size);
                    break;
                case 0x0f:
                    strncpy_P(mcu, PSTR("ATmega328P"), mcu_size);
                    break;
                case 0x14:
                    strncpy_P(mcu, PSTR("ATmega328-PU"), mcu_size);
                    break;
                case 0x16:
                    strncpy_P(mcu, PSTR("ATmega328PB"), mcu_size);
                    break;
            }
        }
    }

    return std::unique_ptr<uint8_t>(buffer);
}
#endif