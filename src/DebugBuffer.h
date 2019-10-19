/**
 * Author: sascha_lammers@gmx.de
 */

#include <Arduino.h>
#include <Printable.h>

template <class T>
class DebugBufferItem : public Printable {
public:
    DebugBufferItem() {
    }

    DebugBufferItem(T data) {
        _data = data;
    }

    virtual ~DebugBufferItem() {
    }

    virtual size_t printTo(Print &p) const {
        size_t written = p.print(_data);
        return written;
    }

private:
    T _data;
};

template <class T, size_t NUM>
class DebugBufferArray : public Printable {
public:
    DebugBufferArray() {
    }

    virtual ~DebugBufferArray() {
    }

    virtual size_t printTo(Print &p) const {
        size_t written = 0;
        for(uint8_t i = 0; i < NUM; i++) {
            if (i != 0) {
                written += p.print(',');
            }
            written += p.print(_items[i]);
        }
        return written;
    }

    DebugBufferArray &add(uint8_t num, const T &item) {
        _items[num] = item;
        return *this;
    }

private:
    T _items[NUM];
};

template <class T>
class DebugBuffer {
public:
    DebugBuffer(size_t size) {
        _size = size;
        _length = 0;
        _position = 0;
        _items = new T[size];
    }
    virtual ~DebugBuffer() {
        delete _items;
    }

    void clear() {
        _position = 0;
        _length = 0;
    }

    void add(const T &item) {
        if (_length < _size) {
            _length++;
        }
        _items[_position] = item;
        _position = (_position + 1) % _size;
    }

    template <class I>
    void add(I item) {
        if (_length < _size) {
            _length++;
        }
        _items[_position] = T(item);
        _position = (_position + 1) % _size;
    }

    void dump(Stream &output, bool copy = true) {
        if (_length == 0) {
            _line(output);
            output.println(F("No data to dump"));
            _line(output);
            return;
        }
        T *temp = _items;
        size_t start;
        if (copy) {
            temp = new T[_length];
            if (!temp) {
                _line(output);
                output.println(F("Cannot allocate memory to dump data"));
                _line(output);
                return;
            }
            cli();
            for(size_t i = 0; i < _length; i++) {
                temp[i] = _items[i];
            }
            start = 0;
            sei();
        }
        else {
            start = (_position + 1) % _size;
        }

        _line(output);

        size_t num = _length;
        while(num--) {
            temp[start].printTo(output);
            output.println();
            start = (start + 1) % _size;
        }

        _line(output);

        if (temp != _items) {
            delete temp;
        }
    }

private:
    void _line(Stream &output) {
        uint8_t count = 76;
        while(count--) {
            output.print('-');
        }
        output.println();
    }

    size_t _position;
    size_t _size;
    size_t _length;
    T *_items;
};
