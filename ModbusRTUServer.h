#ifndef MODBUS_RTUSERVER_H
#define MODBUS_RTUSERVER_H

#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <functional>
#ifdef ARDUINO
#include <Arduino.h>
#endif

// How long the transmitter gets after its enable pin, on the boards that wait for it
#ifndef MODBUS_TX_ENABLE_DELAY_US
#define MODBUS_TX_ENABLE_DELAY_US 1000
#endif

// A function code is left out with -DMODBUS_DISABLE_<name>. Its table stays as long as another one uses it.
#if !defined(MODBUS_DISABLE_READ_COILS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_COIL) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_COILS)
#define MODBUS_HAS_COILS
#endif
#if !defined(MODBUS_DISABLE_READ_HOLDING_REGISTERS) || !defined(MODBUS_DISABLE_WRITE_SINGLE_REGISTER) || !defined(MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS)
#define MODBUS_HAS_HOLDING_REGISTERS
#endif
#ifndef MODBUS_DISABLE_READ_DISCRETE_INPUTS
#define MODBUS_HAS_DISCRETE_INPUTS
#endif
#ifndef MODBUS_DISABLE_READ_INPUT_REGISTERS
#define MODBUS_HAS_INPUT_REGISTERS
#endif

class ModbusRTUServer {
public:
    ModbusRTUServer() = default;
    ~ModbusRTUServer();
    ModbusRTUServer(const ModbusRTUServer&) = delete;  // it owns its tables
    ModbusRTUServer& operator=(const ModbusRTUServer&) = delete;

    /// The serial line the server talks on. On Arduino, begin() also takes any Stream (e.g. &Serial); elsewhere
    /// (ESP-IDF, or a PC for tests) implement this for your UART.
    class Port {
    public:
        virtual ~Port() = default;
        virtual size_t read(uint8_t* data, size_t maxLength) = 0;  ///< what's waiting, up to maxLength, without waiting
        virtual size_t write(const uint8_t* data, size_t length) = 0;
        virtual void flush() = 0;                                  ///< returns once the last byte is sent
    };

    enum FunctionCode : uint8_t {
        READ_COILS = 0x01,
        READ_DISCRETE_INPUTS = 0x02,
        READ_HOLDING_REGISTERS = 0x03,
        READ_INPUT_REGISTERS = 0x04,
        WRITE_SINGLE_COIL = 0x05,
        WRITE_SINGLE_REGISTER = 0x06,
        WRITE_MULTIPLE_COILS = 0x0F,
        WRITE_MULTIPLE_REGISTERS = 0x10,
        REPORT_SERVER_ID = 0x11,
    };

    enum Exception : uint8_t {
        ILLEGAL_FUNCTION = 0x01,
        ILLEGAL_DATA_ADDRESS = 0x02,
        ILLEGAL_DATA_VALUE = 0x03,
    };

    /// Set up the server. The UART itself is the caller's to configure; the tables are made here, zeroed, and
    /// the handlers below installed as the callbacks, so assign your own after this.
    /// @param port The serial line.
    /// @param baud Its rate, which sets the silence that ends a frame.
    /// @param serverId Modbus server ID.
    /// @param txEnablePin Pin used for TX enable, -1 for none.
    /// @param txEnableActiveHigh The pin is high while transmitting.
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

    // The tables, as long as they're in use: read and write them in place or through the accessors below
#ifdef MODBUS_HAS_COILS
    bool* coils = nullptr;
#endif
#ifdef MODBUS_HAS_HOLDING_REGISTERS
    uint16_t* holdingRegisters = nullptr;
#endif
#ifdef MODBUS_HAS_DISCRETE_INPUTS
    bool* discreteInputs = nullptr;
#endif
#ifdef MODBUS_HAS_INPUT_REGISTERS
    uint16_t* inputRegisters = nullptr;
#endif

    // ------------------------------------------------------------------
    // Callbacks. begin() installs the handlers; replace one, or wrap it by capturing it first.
    // ------------------------------------------------------------------
    using CallbackType = std::function<void(uint8_t* data, size_t length, bool broadcast)>;

    // Optional custom framing. When set, update() hands the whole accumulated RX buffer to this callback instead
    // of delimiting by interframe timeout: once per update(), and whenever the buffer fills up while reading, so
    // the same bytes can come by more than once. The callback extracts what complete frames it can (e.g. via
    // checkCrc() + processFrame()) and returns how many bytes it consumed; update() keeps the rest for next time.
    // Useful for buses that ignore Modbus interframe timing.
    using BufferHandlerType = std::function<size_t(uint8_t* data, size_t length)>;

#ifndef MODBUS_DISABLE_READ_COILS
    CallbackType onReadCoils;
#endif
#ifndef MODBUS_DISABLE_READ_DISCRETE_INPUTS
    CallbackType onReadDiscreteInputs;
#endif
#ifndef MODBUS_DISABLE_READ_HOLDING_REGISTERS
    CallbackType onReadHoldingRegisters;
#endif
#ifndef MODBUS_DISABLE_READ_INPUT_REGISTERS
    CallbackType onReadInputRegisters;
#endif
#ifndef MODBUS_DISABLE_REPORT_SERVER_ID
    CallbackType onReportServerID;
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_COIL
    CallbackType onWriteSingleCoil;
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_REGISTER
    CallbackType onWriteSingleRegister;
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_COILS
    CallbackType onWriteMultipleCoils;
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS
    CallbackType onWriteMultipleRegisters;
#endif

    CallbackType onInvalidFunction;  // answers ILLEGAL_FUNCTION unless replaced

    CallbackType onInvalidCrc;  // these two are for watching the line, nothing is answered

    CallbackType onInvalidServer;

    BufferHandlerType onRawBuffer;

    /// Send a frame: the CRC goes in data[length] and data[length + 1], so leave two bytes of room. Nothing
    /// goes out for a broadcast.
    void sendResponse(uint8_t* data, size_t length, bool broadcast);
    void sendException(uint8_t function, uint8_t exceptionCode, bool broadcast);
    void setInterframeTimeout(uint32_t timeoutUs);  ///< the silence that ends a frame, from the baud rate until now
    void setServerId(uint8_t serverId);

    // Some RS485 transceivers keep their receiver on while transmitting, so everything we send comes straight
    // back in on RX. With the echo filter on, bytes that match what we just sent are held back, and dropped once
    // the whole frame has come back. Anything else is passed on untouched. On by default.
    // A request identical to a reply we just sent, as a repeated write single coil or register is, is taken for
    // the echo for 50 ms after the reply. Turn the filter off on a transceiver that doesn't echo.
    void setEchoFilter(bool enabled);

    // True once a frame we sent has come back to us. A write single coil or register reply doesn't count, the
    // master could be repeating its request.
    bool echoDetected() const;

    // Returns true if the trailing CRC-16 of the given frame is valid. Useful for an onRawBuffer handler that
    // needs to detect frame boundaries by CRC.
    bool checkCrc(const uint8_t* data, size_t length);

    // Validate and dispatch a single, already-delimited frame (CRC, server-id and function-code routing).
    // Intended to be called from an onRawBuffer handler.
    void processFrame(uint8_t* data, size_t length);

    // ------------------------------------------------------------------
    // Public Accessors for Data
    // ------------------------------------------------------------------
#ifdef MODBUS_HAS_COILS
    bool getCoil(uint16_t index) const;
    void setCoil(uint16_t index, bool value);
#endif
#ifdef MODBUS_HAS_HOLDING_REGISTERS
    uint16_t getHoldingRegister(uint16_t index) const;
    void setHoldingRegister(uint16_t index, uint16_t value);
#endif
#ifdef MODBUS_HAS_DISCRETE_INPUTS
    bool getDiscreteInput(uint16_t index) const;
    void setDiscreteInput(uint16_t index, bool value);
#endif
#ifdef MODBUS_HAS_INPUT_REGISTERS
    uint16_t getInputRegister(uint16_t index) const;
    void setInputRegister(uint16_t index, uint16_t value);
#endif

private:
#ifdef ARDUINO
    class StreamPort : public Port {  // a Stream as a Port
    public:
        Stream* stream = nullptr;
        size_t read(uint8_t* data, size_t maxLength) override {
            int n = stream->available();  // readBytes() would wait for the rest
            return n > 0 ? stream->readBytes(data, std::min((size_t) n, maxLength)) : 0;
        }
        size_t write(const uint8_t* data, size_t length) override { return stream->write(data, length); }
        void flush() override { stream->flush(); }
    };
    StreamPort streamPort_;
#endif
    Port* port_ = nullptr;
    uint8_t serverId_ = 1;
    size_t numCoils_ = 0;
    size_t numHoldingRegisters_ = 0;
    size_t numDiscreteInputs_ = 0;
    size_t numInputRegisters_ = 0;
    int16_t txEnablePin_ = -1;
    bool txEnableActiveHigh_ = true;

    static const size_t MAX_FRAME = 256;   // a Modbus RTU frame, CRC and all
    static const size_t BUFFER_SIZE = 512; // what gathers before custom framing has to take some
    static const size_t READ_CHUNK = 64;   // a port read at a time, off the stack
    uint8_t buffer_[BUFFER_SIZE];
    size_t bufferIndex_ = 0;
    uint32_t lastByteTime_ = 0;
    uint32_t interframeTimeout_ = 1750;

    // The spec's most in one request
    static const uint16_t MAX_READ_BITS = 2000;
    static const uint16_t MAX_READ_REGISTERS = 125;
    static const uint16_t MAX_WRITE_BITS = 1968;
    static const uint16_t MAX_WRITE_REGISTERS = 123;

    static const uint16_t crcTable_[256];

    // Frames we sent and expect to see again if the transceiver echoes
    static const uint8_t ECHO_MAX_FRAMES = 4;
    static const uint32_t ECHO_TIMEOUT_US = 50000;
    uint8_t echo_[MAX_FRAME];
    size_t echoLength_ = 0;
    size_t echoPos_ = 0;
    size_t echoFrameStart_ = 0;
    size_t echoFrameEnds_[ECHO_MAX_FRAMES];
    uint8_t echoFrameCount_ = 0;
    uint8_t echoFrameIndex_ = 0;
    uint32_t echoTime_ = 0;
    bool echoFilter_ = true;
    bool echoDetected_ = false;

    uint16_t computeCrc(const uint8_t* data, size_t length);
    size_t room() const;
    void storeByte(uint8_t b);
    void handOver();
    void expectEcho(const uint8_t* data, size_t length);
    bool filterEcho(uint8_t b);
    void releaseEcho();
    void clearEcho();
    void readBits(const bool* table, size_t count, uint8_t* data, size_t length, bool broadcast);
    void readRegisters(const uint16_t* table, size_t count, uint8_t* data, size_t length, bool broadcast);

protected:
#ifndef MODBUS_DISABLE_READ_COILS
    void handleReadCoils(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_READ_DISCRETE_INPUTS
    void handleReadDiscreteInputs(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_READ_HOLDING_REGISTERS
    void handleReadHoldingRegisters(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_READ_INPUT_REGISTERS
    void handleReadInputRegisters(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_REPORT_SERVER_ID
    void handleReportServerID(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_COIL
    void handleWriteSingleCoil(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_WRITE_SINGLE_REGISTER
    void handleWriteSingleRegister(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_COILS
    void handleWriteMultipleCoils(uint8_t* data, size_t length, bool broadcast);
#endif
#ifndef MODBUS_DISABLE_WRITE_MULTIPLE_REGISTERS
    void handleWriteMultipleRegisters(uint8_t* data, size_t length, bool broadcast);
#endif
};

#endif  // MODBUS_RTUSERVER_H
