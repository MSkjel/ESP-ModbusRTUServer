#ifndef MODBUS_RTUSERVER_H
#define MODBUS_RTUSERVER_H

#include <stddef.h>
#include <stdint.h>
#include <functional>
#ifdef ARDUINO
#include <Arduino.h>
#endif

#define MODBUS_TX_ENABLE_DELAY_US 1000

class ModbusRTUServer {
public:
    ModbusRTUServer();
    ~ModbusRTUServer();

    /// The serial line the server talks on. On Arduino, begin() also takes any Stream (e.g. &Serial); elsewhere
    /// (ESP-IDF, or a PC for tests) implement this for your UART.
    class Port {
    public:
        virtual ~Port() = default;
        virtual size_t read(uint8_t* data, size_t max) = 0;           ///< what's waiting, up to max bytes, without waiting
        virtual size_t write(const uint8_t* data, size_t length) = 0;
        virtual void flush() = 0;                                     ///< returns once the last byte is sent
    };

    /// Initialize the Modbus server.
    /// @param port The serial line.
    /// @param baud Baud rate.
    /// @param serverId Modbus server ID.
    /// @param txEnablePin Pin used for TX Enable.
    /// @param txEnableActiveHigh TX enmable is high.
    /// @param numCoils Number of coils.
    /// @param numHoldingRegisters Number of holding registers.
    /// @param numDiscreteInputs Number of discrete inputs (for FC 0x02). Default 0.
    /// @param numInputRegisters Number of input registers (for FC 0x04). Default 0.
    void begin(Port* port, uint32_t baud, uint8_t serverId,
                int16_t txEnablePin, bool txEnableActiveHigh,
                size_t numCoils, size_t numHoldingRegisters,
                size_t numDiscreteInputs = 0, size_t numInputRegisters = 0);

#ifdef ARDUINO
    /// The same on an Arduino Stream (e.g. &Serial)
    void begin(Stream* stream, uint32_t baud, uint8_t serverId,
                int16_t txEnablePin, bool txEnableActiveHigh,
                size_t numCoils, size_t numHoldingRegisters,
                size_t numDiscreteInputs = 0, size_t numInputRegisters = 0);
#endif

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

    // Optional custom framing. When set, update() hands the whole accumulated RX
    // buffer to this callback instead of delimiting by interframe timeout. The
    // callback extracts what complete frames it can (e.g. via checkCrc() +
    // processFrame()) and returns how many bytes it consumed; update() keeps the
    // rest for next time. Useful for buses that ignore Modbus interframe timing.
    using BufferHandlerType = std::function<size_t(uint8_t* data, size_t length)>;

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

    BufferHandlerType onRawBuffer;

    void sendResponse(uint8_t* data, size_t length, bool broadcast);
    void sendException(uint8_t function, uint8_t exceptionCode, bool broadcast);
    void setInterframeTimeout(uint32_t timeout);
    void setServerId(uint8_t serverId);

    // Some RS485 transceivers keep their receiver on while transmitting, so
    // everything we send comes straight back in on RX. With the echo filter on,
    // bytes that match what we just sent are held back, and dropped once the
    // whole frame has come back. Anything else is passed on untouched. On by
    // default.
    void setEchoFilter(bool enabled);

    // True once a frame we sent has come back to us.
    bool echoDetected() const;

    // Returns true if the trailing CRC-16 of the given frame is valid. Useful for
    // an onRawBuffer handler that needs to detect frame boundaries by CRC.
    bool checkCrc(uint8_t* data, size_t length);

    // Validate and dispatch a single, already-delimited frame (CRC, server-id and
    // function-code routing). Intended to be called from an onRawBuffer handler.
    void processFrame(uint8_t* data, size_t length);

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
#ifdef ARDUINO
    class StreamPort : public Port {  // a Stream as a Port
    public:
        Stream* stream = nullptr;
        size_t read(uint8_t* data, size_t max) override {
            int n = stream->available();
            if (n <= 0)
                return 0;
            return stream->readBytes(data, (size_t) n < max ? (size_t) n : max);
        }
        size_t write(const uint8_t* data, size_t length) override { return stream->write(data, length); }
        void flush() override { stream->flush(); }
    };
    StreamPort streamPort_;
#endif
    Port* port_;
    uint32_t baud_;
    uint8_t serverId_;
    size_t numCoils_;
    size_t numHoldingRegisters_;
    size_t numDiscreteInputs_;
    size_t numInputRegisters_;
    int16_t txEnablePin_;
    bool txEnableActiveHigh_;

    static const size_t BUFFER_SIZE = 512;
    uint8_t buffer_[BUFFER_SIZE];
    size_t bufferIndex_;
    uint32_t lastByteTime_;
    uint32_t interFrameTimeout_;

    static const uint16_t crcTable_[256];

    // Frames we sent and expect to see again if the transceiver echoes
    static const size_t ECHO_SIZE = 256;
    static const uint8_t ECHO_MAX_FRAMES = 4;
    static const uint32_t ECHO_TIMEOUT_US = 50000;
    uint8_t echo_[ECHO_SIZE];
    size_t echoLength_;
    size_t echoPos_;
    size_t echoFrameStart_;
    size_t echoFrameEnds_[ECHO_MAX_FRAMES];
    uint8_t echoFrameCount_;
    uint8_t echoFrameIndex_;
    uint32_t echoTime_;
    bool echoFilter_;
    bool echoDetected_;

    uint16_t computeCrc(uint8_t* data, size_t length);
    void processRequest(uint8_t* data, size_t length);
    void storeByte(uint8_t b);
    void handOver();
    void expectEcho(const uint8_t* data, size_t length);
    bool filterEcho(uint8_t b);
    void releaseEcho();
    void clearEcho();

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
