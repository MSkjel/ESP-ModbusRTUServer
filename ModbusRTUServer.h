#ifndef MODBUS_RTUSERVER_H
#define MODBUS_RTUSERVER_H

#include <Arduino.h>
#include <functional>

#define MODBUS_TX_ENABLE_DELAY_US 1000

class ModbusRTUServer {
public:
    ModbusRTUServer();
    ~ModbusRTUServer();

    /// Initialize the Modbus server.
    /// @param stream Pointer to the Stream object (e.g. &Serial).
    /// @param baud Baud rate.
    /// @param serverId Modbus server ID.
    /// @param numCoils Number of coils.
    /// @param numHoldingRegisters Number of holding registers.
    /// @param numDiscreteInputs Number of discrete inputs (for FC 0x02). Default 0.
    /// @param numInputRegisters Number of input registers (for FC 0x04). Default 0.
    void begin(Stream* stream, uint32_t baud, uint8_t serverId,
                int16_t txEnablePin = -1, bool txEnableActiveHigh = true,
                size_t numCoils = 0, size_t numHoldingRegisters = 0,
                size_t numDiscreteInputs = 0, size_t numInputRegisters = 0);

    void update();

    #if !defined(MODBUS_DISABLE_READ_COILS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
    bool* coils;
    #endif

    #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
    uint16_t* holdingRegisters;
    #endif

    #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
    bool* discreteInputs;
    #endif

    #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
    uint16_t* inputRegisters;
    #endif

    // ------------------------------------------------------------------
    // Callback function type and callback members.
    // ------------------------------------------------------------------
    using CallbackType = std::function<void(uint8_t* data, size_t length, bool broadcast)>;

    #if !defined(MODBUS_DISABLE_READ_COILS)
    CallbackType onReadCoils;
    #endif

    #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
    CallbackType onReadDiscreteInputs;
    #endif

    #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS)
    CallbackType onReadHoldingRegisters;
    #endif

    #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
    CallbackType onReadInputRegisters;
    #endif

    #if !defined(MODBUS_DISABLE_REPORT_SERVER_ID)
    CallbackType onReportServerID;
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL)
    CallbackType onWriteSingleCoil;
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER)
    CallbackType onWriteSingleRegister;
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
    CallbackType onWriteMultipleCoils;
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
    CallbackType onWriteMultipleRegisters;
    #endif

    CallbackType onInvalidFunction;

    CallbackType onInvalidCrc;

    CallbackType onInvalidServer;

    void sendResponse(uint8_t* data, size_t length, bool broadcast);
    void sendException(uint8_t function, uint8_t exceptionCode, bool broadcast);
    void setInterframeTimeout(uint32_t timeout);
    void setServerId(uint8_t serverId);

    // ------------------------------------------------------------------
    // Public Accessors for Data
    // ------------------------------------------------------------------
    #if !defined(MODBUS_DISABLE_READ_COILS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
    bool getCoil(uint16_t index) const;
    void setCoil(uint16_t index, bool value);
    #endif

    #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
    uint16_t getHoldingRegister(uint16_t index) const;
    void setHoldingRegister(uint16_t index, uint16_t value);
    #endif

    #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
    bool getDiscreteInput(uint16_t index) const;
    void setDiscreteInput(uint16_t index, bool value);
    #endif

    #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
    uint16_t getInputRegister(uint16_t index) const;
    void setInputRegister(uint16_t index, uint16_t value);
    #endif

private:
    Stream* stream_;
    uint32_t baud_;
    uint8_t serverId_;
    size_t numCoils_;
    size_t numHoldingRegisters_;
    size_t numDiscreteInputs_;
    size_t numInputRegisters_;
    uint16_t txEnablePin_;
    bool txEnableActiveHigh_;

    static const size_t BUFFER_SIZE = 256;
    uint8_t buffer_[BUFFER_SIZE];
    size_t bufferIndex_;
    unsigned long lastByteTime_;
    uint32_t interFrameTimeout_;

    static const uint16_t crcTable_[256];

    uint16_t computeCrc(uint8_t* data, size_t length);
    void processRequest(uint8_t* data, size_t length);

protected:
    #if !defined(MODBUS_DISABLE_READ_COILS)
    void handleReadCoils(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_READ_DISCRETE_INPUTS)
    void handleReadDiscreteInputs(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS)
    void handleReadHoldingRegisters(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_READ_INPUT_REGISTERS)
    void handleReadInputRegisters(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_REPORT_SERVER_ID)
    void handleReportServerID(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL)
    void handleWriteSingleCoil(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER)
    void handleWriteSingleRegister(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
    void handleWriteMultipleCoils(uint8_t* data, size_t length, bool broadcast);
    #endif

    #if !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
    void handleWriteMultipleRegisters(uint8_t* data, size_t length, bool broadcast);
    #endif
};

#endif  // MODBUS_RTUSERVER_H
