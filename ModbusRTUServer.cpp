#include "ModbusRTUServer.h"
#include <string.h>

// What the library needs from the platform: the time in microseconds, the TX enable pin, and a pause after
// enabling the transmitter. On Arduino, on ESP-IDF without Arduino, or anywhere else (a PC, for tests: no pins).
#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
#if defined(ARDUINO)
static uint32_t nowUs() { return micros(); }
static void pinInit(int16_t pin) { pinMode(pin, OUTPUT); }
static void pinSet(int16_t pin, bool high) { digitalWrite(pin, high ? HIGH : LOW); }
#elif defined(ESP_PLATFORM)
#include "driver/gpio.h"
#include "esp_timer.h"
static uint32_t nowUs() { return (uint32_t) esp_timer_get_time(); }
static void pinInit(int16_t pin) {
    gpio_reset_pin((gpio_num_t) pin);
    gpio_set_direction((gpio_num_t) pin, GPIO_MODE_OUTPUT);
}
static void pinSet(int16_t pin, bool high) { gpio_set_level((gpio_num_t) pin, high); }
#else
#include <chrono>
static uint32_t nowUs() {
    return (uint32_t) std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static void pinInit(int16_t) {}
static void pinSet(int16_t, bool) {}
#endif

// After enabling the transmitter: an ESP32 lets other tasks run, which is all the time its transceiver needs,
// other boards wait MODBUS_TX_ENABLE_DELAY_US for theirs
static void txSettle() {
#if defined(ESP_PLATFORM)
    taskYIELD();
#elif defined(ARDUINO)
    delayMicroseconds(MODBUS_TX_ENABLE_DELAY_US);
#endif
}

// Outside Arduino the CRC table is plain const data
#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef pgm_read_word
#define pgm_read_word(p) (*(const uint16_t*) (p))
#endif

const uint16_t PROGMEM ModbusRTUServer::crcTable_[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

ModbusRTUServer::~ModbusRTUServer() {
#ifdef MODBUS_HAS_COILS
    delete[] coils;
#endif
#ifdef MODBUS_HAS_HOLDING_REGISTERS
    delete[] holdingRegisters;
#endif
#ifdef MODBUS_HAS_DISCRETE_INPUTS
    delete[] discreteInputs;
#endif
#ifdef MODBUS_HAS_INPUT_REGISTERS
    delete[] inputRegisters;
#endif
}

#ifdef ARDUINO
void ModbusRTUServer::begin(Stream* stream, uint32_t baud, uint8_t serverId,
                            int16_t txEnablePin, bool txEnableActiveHigh,
                            size_t numCoils, size_t numHoldingRegisters,
                            size_t numDiscreteInputs, size_t numInputRegisters) {
    streamPort_.stream = stream;
    begin(&streamPort_, baud, serverId, txEnablePin, txEnableActiveHigh, numCoils, numHoldingRegisters,
          numDiscreteInputs, numInputRegisters);
}
#endif

// A table of n zeroes in place of the one there, if any
template <typename T>
static void makeTable(T*& t, size_t n) {
    delete[] t;
    t = n ? new T[n]() : nullptr;
}

void ModbusRTUServer::begin(Port* port, uint32_t baud, uint8_t serverId,
                            int16_t txEnablePin, bool txEnableActiveHigh,
                            size_t numCoils, size_t numHoldingRegisters,
                            size_t numDiscreteInputs, size_t numInputRegisters) {
    port_ = port;
    serverId_ = serverId;
    txEnablePin_ = txEnablePin;
    txEnableActiveHigh_ = txEnableActiveHigh;

    if (txEnablePin_ >= 0) {
        pinInit(txEnablePin_);
        pinSet(txEnablePin_, !txEnableActiveHigh_);
    }

#ifndef MODBUS_DISABLE_READ_COILS
    onReadCoils = [this](uint8_t* data, size_t length, bool broadcast) { handleReadCoils(data, length, broadcast); };
#endif
#ifndef MODBUS_DISABLE_READ_DISCRETE_INPUTS
    onReadDiscreteInputs = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReadDiscreteInputs(data, length, broadcast);
    };
#endif
#ifndef MODBUS_DISABLE_READ_HOLDING_REGISTERS
    onReadHoldingRegisters = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReadHoldingRegisters(data, length, broadcast);
    };
#endif
#ifndef MODBUS_DISABLE_READ_INPUT_REGISTERS
    onReadInputRegisters = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReadInputRegisters(data, length, broadcast);
    };
#endif
#ifndef MODBUS_DISABLE_REPORT_SERVER_ID
    onReportServerID = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReportServerID(data, length, broadcast);
    };
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_COIL
    onWriteSingleCoil = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteSingleCoil(data, length, broadcast);
    };
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_REGISTER
    onWriteSingleRegister = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteSingleRegister(data, length, broadcast);
    };
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_COILS
    onWriteMultipleCoils = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteMultipleCoils(data, length, broadcast);
    };
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS
    onWriteMultipleRegisters = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteMultipleRegisters(data, length, broadcast);
    };
#endif
    onInvalidFunction = [this](uint8_t* data, size_t, bool broadcast) {
        sendException(data[1], ILLEGAL_FUNCTION, broadcast);
    };

    numCoils_ = numCoils;
    numHoldingRegisters_ = numHoldingRegisters;
    numDiscreteInputs_ = numDiscreteInputs;
    numInputRegisters_ = numInputRegisters;
#ifdef MODBUS_HAS_COILS
    makeTable(coils, numCoils);
#endif
#ifdef MODBUS_HAS_HOLDING_REGISTERS
    makeTable(holdingRegisters, numHoldingRegisters);
#endif
#ifdef MODBUS_HAS_DISCRETE_INPUTS
    makeTable(discreteInputs, numDiscreteInputs);
#endif
#ifdef MODBUS_HAS_INPUT_REGISTERS
    makeTable(inputRegisters, numInputRegisters);
#endif

    // A frame ends with 3.5 characters of silence, 11 bits each, or a fixed 1.75 ms above 19200 baud
    bufferIndex_ = 0;
    lastByteTime_ = nowUs();
    interframeTimeout_ = baud > 19200 || baud == 0 ? 1750 : (uint32_t) (3.5f * (11 * 1000000UL / baud));
}

void ModbusRTUServer::update() {
    if (!port_)
        return;

    // Read at most what the buffer has room for, less the bytes held back as a possible echo, which go in first
    // if it turns out not to be one. With custom framing a full buffer goes to the handler for room; without it
    // the rest stays in the port until the frame is processed.
    uint8_t chunk[READ_CHUNK];
    size_t n;
    while (room() > 0 && (n = port_->read(chunk, std::min(sizeof chunk, room()))) > 0) {
        lastByteTime_ = nowUs();

        for (size_t i = 0; i < n; i++) {
            if (!filterEcho(chunk[i]))
                storeByte(chunk[i]);
        }

        if (room() == 0 && onRawBuffer)
            handOver();
    }

    // Our echo should be back long before this. If it isn't, the bytes we held
    // back weren't an echo after all, so hand them on.
    if (echoLength_ > 0 && (nowUs() - echoTime_) > ECHO_TIMEOUT_US + interframeTimeout_)
        releaseEcho();

    if (bufferIndex_ == 0)
        return;

    if (onRawBuffer) {
        handOver();
        return;
    }

    if ((nowUs() - lastByteTime_) >= interframeTimeout_) {
        processFrame(buffer_, bufferIndex_);
        bufferIndex_ = 0;
    }
}

uint16_t ModbusRTUServer::computeCrc(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
        crc = (crc >> 8) ^ pgm_read_word(&crcTable_[(uint8_t) (crc ^ data[i])]);
    return crc;
}

void ModbusRTUServer::sendResponse(uint8_t* data, size_t length, bool broadcast) {
    if (broadcast || !port_)
        return;

    uint16_t crc = computeCrc(data, length);
    data[length] = crc & 0xFF;
    data[length + 1] = (crc >> 8) & 0xFF;

    if (txEnablePin_ >= 0) {
        pinSet(txEnablePin_, txEnableActiveHigh_);
        txSettle();
    }

    port_->write(data, length + 2);
    port_->flush();

    if (txEnablePin_ >= 0)
        pinSet(txEnablePin_, !txEnableActiveHigh_);

    expectEcho(data, length + 2);
}

void ModbusRTUServer::sendException(uint8_t function, uint8_t exceptionCode, bool broadcast) {
    if (broadcast)
        return;

    // Through sendResponse, so it gets the CRC, the TX enable pin and the flush
    uint8_t response[5] = {serverId_, (uint8_t) (function | 0x80), exceptionCode};
    sendResponse(response, 3, broadcast);
}

void ModbusRTUServer::setInterframeTimeout(uint32_t timeoutUs) {
    interframeTimeout_ = timeoutUs;
}

void ModbusRTUServer::setServerId(uint8_t serverId) {
    serverId_ = serverId;
}

void ModbusRTUServer::setEchoFilter(bool enabled) {
    echoFilter_ = enabled;

    if (!enabled)
        releaseEcho();
}

bool ModbusRTUServer::echoDetected() const {
    return echoDetected_;
}

// What the buffer takes: the room left, less what a possible echo holds back
size_t ModbusRTUServer::room() const {
    size_t held = echoPos_ - echoFrameStart_;
    size_t left = BUFFER_SIZE - bufferIndex_;
    return left > held ? left - held : 0;
}

// Custom framing: hand over the whole buffer and keep whatever it doesn't
// consume. Needed for buses that blast frames with no interframe gap.
void ModbusRTUServer::handOver() {
    size_t consumed = onRawBuffer(buffer_, bufferIndex_);

    if (consumed >= bufferIndex_) {
        bufferIndex_ = 0;
    } else if (consumed > 0) {
        memmove(buffer_, buffer_ + consumed, bufferIndex_ - consumed);
        bufferIndex_ -= consumed;
    } else if (bufferIndex_ >= BUFFER_SIZE) {
        // Buffer full and nothing consumed: drop it so we don't wedge.
        bufferIndex_ = 0;
    }
}

// update() reads no more than there's room for, so this never drops one
void ModbusRTUServer::storeByte(uint8_t b) {
    if (bufferIndex_ < BUFFER_SIZE)
        buffer_[bufferIndex_++] = b;
}

void ModbusRTUServer::expectEcho(const uint8_t* data, size_t length) {
    if (!echoFilter_ || length > MAX_FRAME)
        return;

    // No room behind what we're already waiting for. Can only happen if we keep
    // sending and nothing comes back, so that wasn't an echo anyway. Dropped rather
    // than released, since this can run while an onRawBuffer handler is using the buffer.
    if (echoFrameCount_ >= ECHO_MAX_FRAMES || echoLength_ + length > MAX_FRAME)
        clearEcho();

    memcpy(echo_ + echoLength_, data, length);
    echoLength_ += length;
    echoFrameEnds_[echoFrameCount_++] = echoLength_;
    echoTime_ = nowUs();
}

bool ModbusRTUServer::filterEcho(uint8_t b) {
    if (echoPos_ >= echoLength_)
        return false;

    // Not our echo. Put back what we held, in order, and let this byte follow it.
    if (b != echo_[echoPos_]) {
        releaseEcho();
        return false;
    }

    echoPos_++;
    echoTime_ = nowUs();

    if (echoPos_ == echoFrameEnds_[echoFrameIndex_]) {
        // A whole frame came back, CRC and all, so drop it. Write single coil and
        // write single register answers are identical to the request, so those
        // could also be a master retrying.
        uint8_t function = echo_[echoFrameStart_ + 1];
        if (function != WRITE_SINGLE_COIL && function != WRITE_SINGLE_REGISTER)
            echoDetected_ = true;

        echoFrameStart_ = echoPos_;
        echoFrameIndex_++;

        if (echoFrameIndex_ == echoFrameCount_)
            clearEcho();
    }

    return true;
}

void ModbusRTUServer::releaseEcho() {
    for (size_t i = echoFrameStart_; i < echoPos_; i++)
        storeByte(echo_[i]);

    clearEcho();
}

void ModbusRTUServer::clearEcho() {
    echoLength_ = 0;
    echoPos_ = 0;
    echoFrameStart_ = 0;
    echoFrameCount_ = 0;
    echoFrameIndex_ = 0;
}

bool ModbusRTUServer::checkCrc(const uint8_t* data, size_t length) {
    if (length < 4)
        return false;

    return computeCrc(data, length) == 0;
}

void ModbusRTUServer::processFrame(uint8_t* data, size_t length) {
    if (length < 4)
        return;

    bool broadcast = data[0] == 0;

    if (!broadcast && data[0] != serverId_) {
        if (onInvalidServer)
            onInvalidServer(data, length, broadcast);
        return;
    }

    if (computeCrc(data, length) != 0) {
        if (onInvalidCrc)
            onInvalidCrc(data, length, broadcast);
        return;
    }

    switch (data[1]) {
#ifndef MODBUS_DISABLE_READ_COILS
        case READ_COILS:
            onReadCoils(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_READ_DISCRETE_INPUTS
        case READ_DISCRETE_INPUTS:
            onReadDiscreteInputs(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_READ_HOLDING_REGISTERS
        case READ_HOLDING_REGISTERS:
            onReadHoldingRegisters(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_READ_INPUT_REGISTERS
        case READ_INPUT_REGISTERS:
            onReadInputRegisters(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_COIL
        case WRITE_SINGLE_COIL:
            onWriteSingleCoil(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_REGISTER
        case WRITE_SINGLE_REGISTER:
            onWriteSingleRegister(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_COILS
        case WRITE_MULTIPLE_COILS:
            onWriteMultipleCoils(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS
        case WRITE_MULTIPLE_REGISTERS:
            onWriteMultipleRegisters(data, length, broadcast);
            break;
#endif
#ifndef MODBUS_DISABLE_REPORT_SERVER_ID
        case REPORT_SERVER_ID:
            onReportServerID(data, length, broadcast);
            break;
#endif
        default:
            onInvalidFunction(data, length, broadcast);
            break;
    }
}

// FC 01 and 02: quantity bits from start, eight to a byte, the lowest first
void ModbusRTUServer::readBits(const bool* table, size_t count, uint8_t* data, size_t length, bool broadcast) {
    if (length != 8)
        return;

    uint16_t start = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    if (quantity < 1 || quantity > MAX_READ_BITS) {
        sendException(data[1], ILLEGAL_DATA_VALUE, broadcast);
        return;
    }
    if ((size_t) start + quantity > count) {
        sendException(data[1], ILLEGAL_DATA_ADDRESS, broadcast);
        return;
    }

    uint8_t response[MAX_FRAME] = {serverId_, data[1], (uint8_t) ((quantity + 7) / 8)};
    for (uint16_t i = 0; i < quantity; i++) {
        if (table[start + i])
            response[3 + i / 8] |= 1 << (i % 8);
    }

    sendResponse(response, 3 + response[2], broadcast);
}

// FC 03 and 04: quantity registers from start, high byte first
void ModbusRTUServer::readRegisters(const uint16_t* table, size_t count, uint8_t* data, size_t length,
                                    bool broadcast) {
    if (length != 8)
        return;

    uint16_t start = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    if (quantity < 1 || quantity > MAX_READ_REGISTERS) {
        sendException(data[1], ILLEGAL_DATA_VALUE, broadcast);
        return;
    }
    if ((size_t) start + quantity > count) {
        sendException(data[1], ILLEGAL_DATA_ADDRESS, broadcast);
        return;
    }

    uint8_t response[MAX_FRAME] = {serverId_, data[1], (uint8_t) (quantity * 2)};
    for (uint16_t i = 0; i < quantity; i++) {
        response[3 + 2 * i] = table[start + i] >> 8;
        response[4 + 2 * i] = table[start + i] & 0xFF;
    }

    sendResponse(response, 3 + response[2], broadcast);
}

#ifndef MODBUS_DISABLE_READ_COILS
void ModbusRTUServer::handleReadCoils(uint8_t* data, size_t length, bool broadcast) {
    readBits(coils, numCoils_, data, length, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_READ_DISCRETE_INPUTS
void ModbusRTUServer::handleReadDiscreteInputs(uint8_t* data, size_t length, bool broadcast) {
    readBits(discreteInputs, numDiscreteInputs_, data, length, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_READ_HOLDING_REGISTERS
void ModbusRTUServer::handleReadHoldingRegisters(uint8_t* data, size_t length, bool broadcast) {
    readRegisters(holdingRegisters, numHoldingRegisters_, data, length, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_READ_INPUT_REGISTERS
void ModbusRTUServer::handleReadInputRegisters(uint8_t* data, size_t length, bool broadcast) {
    readRegisters(inputRegisters, numInputRegisters_, data, length, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_REPORT_SERVER_ID
void ModbusRTUServer::handleReportServerID(uint8_t*, size_t, bool broadcast) {
    // Four bytes of our own: a server ID of 1, the run indicator on, and two of nothing
    uint8_t response[9] = {serverId_, REPORT_SERVER_ID, 4, 1, 0xFF, 0, 0};
    sendResponse(response, 7, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_WRITE_SINGLE_COIL
void ModbusRTUServer::handleWriteSingleCoil(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8)
        return;

    uint16_t address = (data[2] << 8) | data[3];
    uint16_t value = (data[4] << 8) | data[5];

    if (value != 0xFF00 && value != 0x0000) {
        sendException(data[1], ILLEGAL_DATA_VALUE, broadcast);
        return;
    }
    if (address >= numCoils_) {
        sendException(data[1], ILLEGAL_DATA_ADDRESS, broadcast);
        return;
    }

    coils[address] = value == 0xFF00;

    sendResponse(data, 6, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_WRITE_SINGLE_REGISTER
void ModbusRTUServer::handleWriteSingleRegister(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8)
        return;

    uint16_t address = (data[2] << 8) | data[3];
    uint16_t value = (data[4] << 8) | data[5];

    if (address >= numHoldingRegisters_) {
        sendException(data[1], ILLEGAL_DATA_ADDRESS, broadcast);
        return;
    }

    holdingRegisters[address] = value;

    sendResponse(data, 6, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_COILS
void ModbusRTUServer::handleWriteMultipleCoils(uint8_t* data, size_t length, bool broadcast) {
    if (length < 7)
        return;

    uint16_t start = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];
    uint8_t byteCount = data[6];

    if (quantity < 1 || quantity > MAX_WRITE_BITS || byteCount != (quantity + 7) / 8 ||
        length < 7 + (size_t) byteCount) {
        sendException(data[1], ILLEGAL_DATA_VALUE, broadcast);
        return;
    }
    if ((size_t) start + quantity > numCoils_) {
        sendException(data[1], ILLEGAL_DATA_ADDRESS, broadcast);
        return;
    }

    for (uint16_t i = 0; i < quantity; i++)
        coils[start + i] = (data[7 + i / 8] >> (i % 8)) & 1;

    sendResponse(data, 6, broadcast);
}
#endif

#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS
void ModbusRTUServer::handleWriteMultipleRegisters(uint8_t* data, size_t length, bool broadcast) {
    if (length < 7)
        return;

    uint16_t start = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];
    uint8_t byteCount = data[6];

    if (quantity < 1 || quantity > MAX_WRITE_REGISTERS || byteCount != quantity * 2 ||
        length < 7 + (size_t) byteCount) {
        sendException(data[1], ILLEGAL_DATA_VALUE, broadcast);
        return;
    }
    if ((size_t) start + quantity > numHoldingRegisters_) {
        sendException(data[1], ILLEGAL_DATA_ADDRESS, broadcast);
        return;
    }

    for (uint16_t i = 0; i < quantity; i++)
        holdingRegisters[start + i] = (data[7 + 2 * i] << 8) | data[8 + 2 * i];

    sendResponse(data, 6, broadcast);
}
#endif

// ------------------------------------------------------------------
// Public Accessors for Data
// ------------------------------------------------------------------
#ifdef MODBUS_HAS_COILS
bool ModbusRTUServer::getCoil(uint16_t index) const {
    return index < numCoils_ && coils ? coils[index] : false;
}

void ModbusRTUServer::setCoil(uint16_t index, bool value) {
    if (index < numCoils_ && coils)
        coils[index] = value;
}
#endif

#ifdef MODBUS_HAS_HOLDING_REGISTERS
uint16_t ModbusRTUServer::getHoldingRegister(uint16_t index) const {
    return index < numHoldingRegisters_ && holdingRegisters ? holdingRegisters[index] : 0;
}

void ModbusRTUServer::setHoldingRegister(uint16_t index, uint16_t value) {
    if (index < numHoldingRegisters_ && holdingRegisters)
        holdingRegisters[index] = value;
}
#endif

#ifdef MODBUS_HAS_DISCRETE_INPUTS
bool ModbusRTUServer::getDiscreteInput(uint16_t index) const {
    return index < numDiscreteInputs_ && discreteInputs ? discreteInputs[index] : false;
}

void ModbusRTUServer::setDiscreteInput(uint16_t index, bool value) {
    if (index < numDiscreteInputs_ && discreteInputs)
        discreteInputs[index] = value;
}
#endif

#ifdef MODBUS_HAS_INPUT_REGISTERS
uint16_t ModbusRTUServer::getInputRegister(uint16_t index) const {
    return index < numInputRegisters_ && inputRegisters ? inputRegisters[index] : 0;
}

void ModbusRTUServer::setInputRegister(uint16_t index, uint16_t value) {
    if (index < numInputRegisters_ && inputRegisters)
        inputRegisters[index] = value;
}
#endif
