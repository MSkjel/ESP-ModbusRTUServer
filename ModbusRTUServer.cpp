#include "ModbusRTUServer.h"

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

ModbusRTUServer::ModbusRTUServer()
    :   coils(nullptr),
        holdingRegisters(nullptr),
        discreteInputs(nullptr),
        inputRegisters(nullptr),
        stream_(nullptr),
        baud_(9600),
        serverId_(1),
        numCoils_(0),
        numHoldingRegisters_(0),
        numDiscreteInputs_(0),
        numInputRegisters_(0),
        bufferIndex_(0),
        lastByteTime_(0),
        interFrameTimeout_(5)
{}

ModbusRTUServer::~ModbusRTUServer() {
    #if !defined(MODBUS_DISABLE_READ_COILS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
    if (coils) {
        delete[] coils;
    }
    #endif
    #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
    if (holdingRegisters) {
        delete[] holdingRegisters;
    }
    #endif
    #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
    if (discreteInputs) {
        delete[] discreteInputs;
    }
    #endif
    #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
    if (inputRegisters) {
        delete[] inputRegisters;
    }
    #endif
}

void ModbusRTUServer::begin(Stream* stream, uint32_t baud, uint8_t serverId,
                            size_t numCoils, size_t numHoldingRegisters,
                            size_t numDiscreteInputs, size_t numInputRegisters, int16_t txEnablePin, bool txEnableActiveHigh) {
    stream_ = stream;
    baud_ = baud;
    serverId_ = serverId;
    numCoils_ = numCoils;
    numHoldingRegisters_ = numHoldingRegisters;
    numDiscreteInputs_ = numDiscreteInputs;
    numInputRegisters_ = numInputRegisters;
    txEnablePin_ = txEnablePin;
    txEnableActiveHigh_ = txEnableActiveHigh;

    if (txEnablePin_ >= 0) {
        pinMode(txEnablePin_, OUTPUT);
        digitalWrite(txEnablePin_, txEnableActiveHigh_ ? LOW : HIGH);
    }

    #if !defined(MODBUS_DISABLE_READ_COILS)
    onReadCoils = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReadCoils(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
    onReadDiscreteInputs = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReadDiscreteInputs(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS)
    onReadHoldingRegisters = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReadHoldingRegisters(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
    onReadInputRegisters = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReadInputRegisters(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_REPORT_SERVER_ID)
    onReportServerID = [this](uint8_t* data, size_t length, bool broadcast) {
        handleReportServerID(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL)
    onWriteSingleCoil = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteSingleCoil(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER)
    onWriteSingleRegister = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteSingleRegister(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
    onWriteMultipleCoils = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteMultipleCoils(data, length, broadcast);
    };
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
    onWriteMultipleRegisters = [this](uint8_t* data, size_t length, bool broadcast) {
        handleWriteMultipleRegisters(data, length, broadcast);
    };
    #endif

    onInvalidFunction = [this](uint8_t* data, size_t length, bool broadcast) {
        sendException(data[1], 0x01, broadcast);
    };

    onInvalidServer = nullptr;

    onInvalidCrc = nullptr;

    #if !defined(MODBUS_DISABLE_READ_COILS)
    if (numCoils > 0) {
        coils = new bool[numCoils];
        
        std::fill_n(coils, numCoils, 0);
    }
    #endif

    #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS)
    if (numHoldingRegisters > 0) {
        holdingRegisters = new uint16_t[numHoldingRegisters];

        std::fill_n(holdingRegisters, numHoldingRegisters, 0);
    }
    #endif

    #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
    if (numDiscreteInputs > 0) {
        discreteInputs = new bool[numDiscreteInputs];

        std::fill_n(discreteInputs, numDiscreteInputs, 0);
    }
    #endif

    #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
    if (numInputRegisters > 0) {
        inputRegisters = new uint16_t[numInputRegisters];

        std::fill_n(inputRegisters, numInputRegisters, 0);
    }
    #endif

    bufferIndex_ = 0;
    lastByteTime_ = micros();
    uint32_t charTimeUs = 11 * 1000000UL / baud;
    interFrameTimeout_ = baud_ > 19200 ? 1750UL : uint32_t(3.5f * charTimeUs);
}

void ModbusRTUServer::update() {
    if (!stream_) 
        return;

    while (stream_->available() > 0) {
        if (bufferIndex_ < BUFFER_SIZE) {
            buffer_[bufferIndex_++] = stream_->read();
            lastByteTime_ = micros();
        } else {
            bufferIndex_ = 0;
        }
    }

    if (bufferIndex_ > 0 && (micros() - lastByteTime_) >= interFrameTimeout_) {
        processRequest(buffer_, bufferIndex_);
        bufferIndex_ = 0;
    }
}

uint16_t ModbusRTUServer::computeCrc(uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    size_t pos = 0;
    
    while (pos < length && ((uintptr_t)(data + pos) & 3) != 0) {
        uint8_t index = crc ^ data[pos];
        crc = (crc >> 8) ^ pgm_read_word(&crcTable_[index]);
        pos++;
    }
    
    while (pos + 4 <= length) {
        uint32_t word = *(uint32_t*)(data + pos);
        
        uint8_t index0 = crc ^ (word & 0xFF);
        crc = (crc >> 8) ^ pgm_read_word(&crcTable_[index0]);
        
        uint8_t index1 = crc ^ ((word >> 8) & 0xFF);
        crc = (crc >> 8) ^ pgm_read_word(&crcTable_[index1]);
        
        uint8_t index2 = crc ^ ((word >> 16) & 0xFF);
        crc = (crc >> 8) ^ pgm_read_word(&crcTable_[index2]);
        
        uint8_t index3 = crc ^ ((word >> 24) & 0xFF);
        crc = (crc >> 8) ^ pgm_read_word(&crcTable_[index3]);
        
        pos += 4;
    }
    
    while (pos < length) {
        uint8_t index = crc ^ data[pos];
        crc = (crc >> 8) ^ pgm_read_word(&crcTable_[index]);
        pos++;
    }
    
    return crc;
}

void ModbusRTUServer::sendResponse(uint8_t* data, size_t length, bool broadcast) {
    if (broadcast || !stream_) 
        return;

    uint16_t crc = computeCrc(data, length);

    data[length] = crc & 0xFF;
    data[length + 1] = (crc >> 8) & 0xFF;

    if (txEnablePin_ >= 0)
    {
        digitalWrite(txEnablePin_, txEnableActiveHigh_ ? HIGH : LOW);
        delayMicroseconds(MODBUS_TX_ENABLE_DELAY_US);
    }

    stream_->write(data, length + 2);
    stream_->flush();

    if (txEnablePin_ >= 0)
    {
        delayMicroseconds(MODBUS_TX_ENABLE_DELAY_US);
        digitalWrite(txEnablePin_, txEnableActiveHigh_ ? LOW : HIGH);
    }
}

void ModbusRTUServer::sendException(uint8_t function, uint8_t exceptionCode, bool broadcast) {
    if (broadcast)
        return;

    uint8_t response[5];
    response[0] = serverId_;
    response[1] = function | 0x80;
    response[2] = exceptionCode;

    uint16_t crc = computeCrc(response, 3);

    response[3] = crc & 0xFF;
    response[4] = (crc >> 8) & 0xFF;

    stream_->write(response, 5);
}

void ModbusRTUServer::setInterframeTimeout(uint32_t timeout) {
    interFrameTimeout_ = timeout;
}

void ModbusRTUServer::setServerId(uint8_t serverId) {
    serverId_ = serverId;
}

inline void ModbusRTUServer::processRequest(uint8_t* data, size_t length) {
    if (length < 4) 
        return;

    uint8_t recvServerId = data[0];
    bool broadcast = (recvServerId == 0);

    if (!broadcast && recvServerId != serverId_)
    {
        if (onInvalidServer) {
            onInvalidServer(data, length, broadcast);
        }
        return;
    }

    if (computeCrc(data, length) != 0) 
    {
        if (onInvalidCrc) {
            onInvalidCrc(data, length, broadcast);
        }
        return;
    }

    uint8_t function = data[1];

    switch (function) {
        #if !defined(MODBUS_DISABLE_READ_COILS)
        case 0x01:
            onReadCoils(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
        case 0x02:
            onReadDiscreteInputs(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS)
        case 0x03:
            onReadHoldingRegisters(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
        case 0x04:
            onReadInputRegisters(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL)
        case 0x05:
            onWriteSingleCoil(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER)
        case 0x06:
            onWriteSingleRegister(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
        case 0x0F:
            onWriteMultipleCoils(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
        case 0x10:
            onWriteMultipleRegisters(data, length, broadcast);
            break;
        #endif
        #if !defined(MODBUS_DISABLE_REPORT_SERVER_ID)
        case 0x11:
            onReportServerID(data, length, broadcast);
            break;
        #endif
        default:
            onInvalidFunction(data, length, broadcast);
            break;
    }
}

#if !defined(MODBUS_DISABLE_READ_COILS)
void ModbusRTUServer::handleReadCoils(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8) 
        return;

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    if (quantity < 1 || quantity > 2000 || (startAddress + quantity) > numCoils_) {
        sendException(data[1], 0x02, broadcast);
        return;
    }

    uint8_t byteCount = (quantity + 7) >> 3;
    uint8_t response[BUFFER_SIZE];
    response[0] = serverId_;
    response[1] = data[1];
    response[2] = byteCount;

    std::fill_n(&response[3], byteCount, 0);

    bool* coilPtr = coils + startAddress;
    uint16_t remaining = quantity;
    uint16_t i = 0;

    while (remaining >= 8) {
        if (coilPtr[0]) response[3 + (i / 8)] |= (1 << (i % 8));
        if (coilPtr[1]) response[3 + ((i + 1) / 8)] |= (1 << ((i + 1) % 8));
        if (coilPtr[2]) response[3 + ((i + 2) / 8)] |= (1 << ((i + 2) % 8));
        if (coilPtr[3]) response[3 + ((i + 3) / 8)] |= (1 << ((i + 3) % 8));
        if (coilPtr[4]) response[3 + ((i + 4) / 8)] |= (1 << ((i + 4) % 8));
        if (coilPtr[5]) response[3 + ((i + 5) / 8)] |= (1 << ((i + 5) % 8));
        if (coilPtr[6]) response[3 + ((i + 6) / 8)] |= (1 << ((i + 6) % 8));
        if (coilPtr[7]) response[3 + ((i + 7) / 8)] |= (1 << ((i + 7) % 8));
        
        coilPtr += 8;
        i += 8;
        remaining -= 8;
    }

    switch (remaining) {
        case 7: if (*coilPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 6: if (*coilPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 5: if (*coilPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 4: if (*coilPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 3: if (*coilPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 2: if (*coilPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 1: if (*coilPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 0: break;
    }

    sendResponse(response, 3 + byteCount, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
void ModbusRTUServer::handleReadDiscreteInputs(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8) 
        return;

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    if (quantity < 1 || quantity > 2000 || (startAddress + quantity) > numDiscreteInputs_) {
        sendException(data[1], 0x02, broadcast);
        return;
    }

    uint8_t byteCount = (quantity + 7) >> 3;
    uint8_t response[BUFFER_SIZE];
    response[0] = serverId_;
    response[1] = data[1];
    response[2] = byteCount;

    std::fill_n(&response[3], byteCount, 0);

    bool* inputPtr = discreteInputs + startAddress;
    uint16_t remaining = quantity;
    uint16_t i = 0;

    while (remaining >= 8) {
        if (inputPtr[0]) response[3 + (i / 8)] |= (1 << (i % 8));
        if (inputPtr[1]) response[3 + ((i + 1) / 8)] |= (1 << ((i + 1) % 8));
        if (inputPtr[2]) response[3 + ((i + 2) / 8)] |= (1 << ((i + 2) % 8));
        if (inputPtr[3]) response[3 + ((i + 3) / 8)] |= (1 << ((i + 3) % 8));
        if (inputPtr[4]) response[3 + ((i + 4) / 8)] |= (1 << ((i + 4) % 8));
        if (inputPtr[5]) response[3 + ((i + 5) / 8)] |= (1 << ((i + 5) % 8));
        if (inputPtr[6]) response[3 + ((i + 6) / 8)] |= (1 << ((i + 6) % 8));
        if (inputPtr[7]) response[3 + ((i + 7) / 8)] |= (1 << ((i + 7) % 8));
        
        inputPtr += 8;
        i += 8;
        remaining -= 8;
    }

    switch (remaining) {
        case 7: if (*inputPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 6: if (*inputPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 5: if (*inputPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 4: if (*inputPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 3: if (*inputPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 2: if (*inputPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 1: if (*inputPtr++) response[3 + (i / 8)] |= (1 << (i % 8)); i++;
        case 0: break;
    }

    sendResponse(response, 3 + byteCount, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS)
void ModbusRTUServer::handleReadHoldingRegisters(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8) 
        return;

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    if (quantity < 1 || quantity > 125 || (startAddress + quantity) > numHoldingRegisters_) {
        sendException(data[1], 0x02, broadcast);
        return;
    }

    uint8_t byteCount = quantity * 2;
    uint8_t response[BUFFER_SIZE];
    response[0] = serverId_;
    response[1] = data[1];
    response[2] = byteCount;

    if (3 + byteCount + 2 > BUFFER_SIZE) {
        sendException(data[1], 0x03, broadcast);
        return;
    }

    uint16_t* regPtr = holdingRegisters + startAddress;
    uint8_t* respPtr = response + 3;
    uint16_t remaining = quantity;

    while (remaining >= 8) {
        respPtr[0]  = regPtr[0] >> 8; respPtr[1]  = regPtr[0] & 0xFF;
        respPtr[2]  = regPtr[1] >> 8; respPtr[3]  = regPtr[1] & 0xFF;
        respPtr[4]  = regPtr[2] >> 8; respPtr[5]  = regPtr[2] & 0xFF;
        respPtr[6]  = regPtr[3] >> 8; respPtr[7]  = regPtr[3] & 0xFF;
        respPtr[8]  = regPtr[4] >> 8; respPtr[9]  = regPtr[4] & 0xFF;
        respPtr[10] = regPtr[5] >> 8; respPtr[11] = regPtr[5] & 0xFF;
        respPtr[12] = regPtr[6] >> 8; respPtr[13] = regPtr[6] & 0xFF;
        respPtr[14] = regPtr[7] >> 8; respPtr[15] = regPtr[7] & 0xFF;
        
        regPtr += 8;
        respPtr += 16;
        remaining -= 8;
    }

    switch (remaining) {
        case 7: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 6: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 5: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 4: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 3: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 2: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 1: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 0: break;
    }

    sendResponse(response, 3 + byteCount, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
void ModbusRTUServer::handleReadInputRegisters(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8) 
        return;

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    if (quantity < 1 || quantity > 125 || (startAddress + quantity) > numInputRegisters_) {
        sendException(data[1], 0x02, broadcast);
        return;
    }

    uint8_t byteCount = quantity * 2;
    uint8_t response[BUFFER_SIZE];
    response[0] = serverId_;
    response[1] = data[1];
    response[2] = byteCount;

    uint16_t* regPtr = inputRegisters + startAddress;
    uint8_t* respPtr = response + 3;
    uint16_t remaining = quantity;

    while (remaining >= 8) {
        respPtr[0]  = regPtr[0] >> 8; respPtr[1]  = regPtr[0] & 0xFF;
        respPtr[2]  = regPtr[1] >> 8; respPtr[3]  = regPtr[1] & 0xFF;
        respPtr[4]  = regPtr[2] >> 8; respPtr[5]  = regPtr[2] & 0xFF;
        respPtr[6]  = regPtr[3] >> 8; respPtr[7]  = regPtr[3] & 0xFF;
        respPtr[8]  = regPtr[4] >> 8; respPtr[9]  = regPtr[4] & 0xFF;
        respPtr[10] = regPtr[5] >> 8; respPtr[11] = regPtr[5] & 0xFF;
        respPtr[12] = regPtr[6] >> 8; respPtr[13] = regPtr[6] & 0xFF;
        respPtr[14] = regPtr[7] >> 8; respPtr[15] = regPtr[7] & 0xFF;
        
        regPtr += 8;
        respPtr += 16;
        remaining -= 8;
    }

    switch (remaining) {
        case 7: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 6: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 5: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 4: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 3: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 2: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 1: *respPtr++ = (*regPtr >> 8); *respPtr++ = (*regPtr++ & 0xFF);
        case 0: break;
    }

    sendResponse(response, 3 + byteCount, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_REPORT_SERVER_ID)
void ModbusRTUServer::handleReportServerID(uint8_t* data, size_t length, bool broadcast) {
    uint8_t response[7];
    response[0] = serverId_;
    response[1] = 0x11;
    response[2] = 4;
    response[3] = 1;
    response[4] = 0;
    response[5] = 0;
    response[6] = 0;

    sendResponse(response, 7, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL)
void ModbusRTUServer::handleWriteSingleCoil(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8) 
        return;

    uint16_t coilAddress = (data[2] << 8) | data[3];
    uint16_t value = (data[4] << 8) | data[5];

    if (coilAddress >= numCoils_ || (value != 0xFF00 && value != 0x0000)) {
        sendException(data[1], 0x03, broadcast);
        return;
    }

    coils[coilAddress] = (value == 0xFF00);

    sendResponse(data, 6, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER)
void ModbusRTUServer::handleWriteSingleRegister(uint8_t* data, size_t length, bool broadcast) {
    if (length != 8) 
        return;

    uint16_t regAddress = (data[2] << 8) | data[3];
    uint16_t value = (data[4] << 8) | data[5];

    if (regAddress >= numHoldingRegisters_) {
        sendException(data[1], 0x02, broadcast);
        return;
    }

    holdingRegisters[regAddress] = value;

    sendResponse(data, 6, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
void ModbusRTUServer::handleWriteMultipleCoils(uint8_t* data, size_t length, bool broadcast) {
    if (length < 7) 
        return;

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];
    uint8_t byteCount = data[6];
    uint8_t expectedByteCount = (quantity + 7) / 8;

    if (quantity < 1 || quantity > 0x07B0 ||
        (startAddress + quantity) > numCoils_ || 
        length < 7 + byteCount ||
        byteCount != expectedByteCount) {
        sendException(data[1], 0x02, broadcast);
        return;
    }

    uint8_t* dataPtr = data + 7;
    bool* coilPtr = coils + startAddress;
    uint16_t remaining = quantity;
    uint16_t i = 0;

    while (remaining >= 8) {
        coilPtr[0] = (dataPtr[i / 8] >> (i % 8)) & 0x01;
        coilPtr[1] = (dataPtr[(i + 1) / 8] >> ((i + 1) % 8)) & 0x01;
        coilPtr[2] = (dataPtr[(i + 2) / 8] >> ((i + 2) % 8)) & 0x01;
        coilPtr[3] = (dataPtr[(i + 3) / 8] >> ((i + 3) % 8)) & 0x01;
        coilPtr[4] = (dataPtr[(i + 4) / 8] >> ((i + 4) % 8)) & 0x01;
        coilPtr[5] = (dataPtr[(i + 5) / 8] >> ((i + 5) % 8)) & 0x01;
        coilPtr[6] = (dataPtr[(i + 6) / 8] >> ((i + 6) % 8)) & 0x01;
        coilPtr[7] = (dataPtr[(i + 7) / 8] >> ((i + 7) % 8)) & 0x01;
        
        coilPtr += 8;
        i += 8;
        remaining -= 8;
    }

    switch (remaining) {
        case 7: *coilPtr++ = (dataPtr[i / 8] >> (i % 8)) & 0x01; i++;
        case 6: *coilPtr++ = (dataPtr[i / 8] >> (i % 8)) & 0x01; i++;
        case 5: *coilPtr++ = (dataPtr[i / 8] >> (i % 8)) & 0x01; i++;
        case 4: *coilPtr++ = (dataPtr[i / 8] >> (i % 8)) & 0x01; i++;
        case 3: *coilPtr++ = (dataPtr[i / 8] >> (i % 8)) & 0x01; i++;
        case 2: *coilPtr++ = (dataPtr[i / 8] >> (i % 8)) & 0x01; i++;
        case 1: *coilPtr++ = (dataPtr[i / 8] >> (i % 8)) & 0x01; i++;
        case 0: break;
    }

    sendResponse(data, 6, broadcast);
}
#endif

#if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
void ModbusRTUServer::handleWriteMultipleRegisters(uint8_t* data, size_t length, bool broadcast) {
    if (length < 7) 
        return;

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];
    uint8_t byteCount = data[6];
    uint8_t expectedByteCount = quantity * 2;

    if (quantity < 1 || quantity > 123 ||
        (startAddress + quantity) > numHoldingRegisters_ ||
        length < 7 + byteCount ||
        byteCount != expectedByteCount) {
        sendException(data[1], 0x02, broadcast);
        return;
    }

    uint8_t* dataPtr = data + 7;
    uint16_t* regPtr = holdingRegisters + startAddress;
    uint16_t remaining = quantity;

    while (remaining >= 8) {
        regPtr[0] = (dataPtr[0] << 8) | dataPtr[1];
        regPtr[1] = (dataPtr[2] << 8) | dataPtr[3];
        regPtr[2] = (dataPtr[4] << 8) | dataPtr[5];
        regPtr[3] = (dataPtr[6] << 8) | dataPtr[7];
        regPtr[4] = (dataPtr[8] << 8) | dataPtr[9];
        regPtr[5] = (dataPtr[10] << 8) | dataPtr[11];
        regPtr[6] = (dataPtr[12] << 8) | dataPtr[13];
        regPtr[7] = (dataPtr[14] << 8) | dataPtr[15];
        
        regPtr += 8;
        dataPtr += 16;
        remaining -= 8;
    }

    switch (remaining) {
        case 7: *regPtr++ = (*dataPtr << 8) | *(dataPtr + 1); dataPtr += 2;
        case 6: *regPtr++ = (*dataPtr << 8) | *(dataPtr + 1); dataPtr += 2;
        case 5: *regPtr++ = (*dataPtr << 8) | *(dataPtr + 1); dataPtr += 2;
        case 4: *regPtr++ = (*dataPtr << 8) | *(dataPtr + 1); dataPtr += 2;
        case 3: *regPtr++ = (*dataPtr << 8) | *(dataPtr + 1); dataPtr += 2;
        case 2: *regPtr++ = (*dataPtr << 8) | *(dataPtr + 1); dataPtr += 2;
        case 1: *regPtr++ = (*dataPtr << 8) | *(dataPtr + 1); dataPtr += 2;
        case 0: break;
    }

    sendResponse(data, 6, broadcast);
}
#endif

// ------------------------------------------------------------------
// Public Accessors for Data
// ------------------------------------------------------------------
#if !defined(MODBUS_DISABLE_READ_COILS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
bool ModbusRTUServer::getCoil(uint16_t index) const {
    if (index >= numCoils_ || !coils)
        return false;

    return coils[index];
}

void ModbusRTUServer::setCoil(uint16_t index, bool value) {
    if (index < numCoils_ && coils)
        coils[index] = value;
}
#endif

#if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
uint16_t ModbusRTUServer::getHoldingRegister(uint16_t index) const {
    if (index >= numHoldingRegisters_ || !holdingRegisters)
        return 0;

    return holdingRegisters[index];
}

void ModbusRTUServer::setHoldingRegister(uint16_t index, uint16_t value) {
    if (index < numHoldingRegisters_ && holdingRegisters)
        holdingRegisters[index] = value;
}
#endif

#if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
bool ModbusRTUServer::getDiscreteInput(uint16_t index) const {
    if (index >= numDiscreteInputs_ || !discreteInputs)
        return false;

    return discreteInputs[index];
}

void ModbusRTUServer::setDiscreteInput(uint16_t index, bool value) {
    if (index < numDiscreteInputs_ && discreteInputs)
        discreteInputs[index] = value;
}
#endif

#if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
uint16_t ModbusRTUServer::getInputRegister(uint16_t index) const {
    if (index >= numInputRegisters_ || !inputRegisters)
        return 0;

    return inputRegisters[index];
}

void ModbusRTUServer::setInputRegister(uint16_t index, uint16_t value) {
    if (index < numInputRegisters_ && inputRegisters)
        inputRegisters[index] = value;
}
#endif