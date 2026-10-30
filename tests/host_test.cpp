// The library on a PC, without Arduino or ESP-IDF: requests in through a fake Port, answers out.
//   make -C tests
#include <chrono>
#include <cstdio>
#include <deque>
#include <thread>
#include <vector>
#include "ModbusRTUServer.h"

static int fails;
#define CHECK(c)                                          \
    do {                                                  \
        if (!(c)) {                                       \
            printf("%s:%d: %s\n", __FILE__, __LINE__, #c); \
            fails++;                                      \
        }                                                 \
    } while (0)

struct FakePort : ModbusRTUServer::Port {
    std::deque<uint8_t> in;
    std::vector<uint8_t> out;
    size_t read(uint8_t* data, size_t maxLength) override {
        size_t n = 0;
        for (; n < maxLength && !in.empty(); n++) {
            data[n] = in.front();
            in.pop_front();
        }
        return n;
    }
    size_t write(const uint8_t* data, size_t length) override {
        out.insert(out.end(), data, data + length);
        return length;
    }
    void flush() override {}
};

static std::vector<uint8_t> withCrc(std::vector<uint8_t> f) {
    uint16_t crc = 0xFFFF;
    for (uint8_t b : f) {
        crc ^= b;
        for (int i = 0; i < 8; i++)
            crc = crc & 1 ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    f.push_back(crc & 0xFF);
    f.push_back(crc >> 8);
    return f;
}

// a frame on the line, then the silence that ends it
static void send(ModbusRTUServer& s, FakePort& p, const std::vector<uint8_t>& frame) {
    p.in.insert(p.in.end(), frame.begin(), frame.end());
    s.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    s.update();
}

// what a request, CRC added, gets answered with
static std::vector<uint8_t> ask(ModbusRTUServer& s, FakePort& p, const std::vector<uint8_t>& request) {
    p.out.clear();
    send(s, p, withCrc(request));
    return p.out;
}

// Every function code, and what's refused
static void requests() {
    ModbusRTUServer s;
    FakePort p;
    s.begin(&p, 115200, 3, -1, true, 32, 16, 16, 16);
    s.setHoldingRegister(1, 0x1234);
    s.setHoldingRegister(2, 0xABCD);
    s.setInputRegister(0, 0x0102);
    s.setCoil(0, true);
    s.setCoil(9, true);

    CHECK(ask(s, p, {3, 0x03, 0, 1, 0, 2}) == withCrc({3, 0x03, 4, 0x12, 0x34, 0xAB, 0xCD}));
    CHECK(ask(s, p, {3, 0x04, 0, 0, 0, 1}) == withCrc({3, 0x04, 2, 0x01, 0x02}));
#ifndef MODBUS_DISABLE_READ_COILS
    CHECK(ask(s, p, {3, 0x01, 0, 0, 0, 10}) == withCrc({3, 0x01, 2, 0x01, 0x02}));
#endif
#ifndef MODBUS_DISABLE_READ_DISCRETE_INPUTS
    s.setDiscreteInput(3, true);
    CHECK(ask(s, p, {3, 0x02, 0, 2, 0, 3}) == withCrc({3, 0x02, 1, 0x02}));
#endif
#ifndef MODBUS_DISABLE_REPORT_SERVER_ID
    CHECK(ask(s, p, {3, 0x11}) == withCrc({3, 0x11, 4, 1, 0xFF, 0, 0}));
#endif

    // the writes answer with their request
    CHECK(ask(s, p, {3, 0x06, 0, 5, 0x00, 0x2A}) == withCrc({3, 0x06, 0, 5, 0x00, 0x2A}));
    CHECK(s.getHoldingRegister(5) == 0x2A);
    CHECK(ask(s, p, {3, 0x05, 0, 4, 0xFF, 0x00}) == withCrc({3, 0x05, 0, 4, 0xFF, 0x00}));
    CHECK(s.getCoil(4));
    CHECK(ask(s, p, {3, 0x10, 0, 6, 0, 2, 4, 0xAA, 0xBB, 0xCC, 0xDD}) == withCrc({3, 0x10, 0, 6, 0, 2}));
    CHECK(s.getHoldingRegister(6) == 0xAABB && s.getHoldingRegister(7) == 0xCCDD);
    CHECK(ask(s, p, {3, 0x0F, 0, 8, 0, 10, 2, 0x05, 0x02}) == withCrc({3, 0x0F, 0, 8, 0, 10}));
    CHECK(s.getCoil(8) && !s.getCoil(9) && s.getCoil(10) && s.getCoil(17));

    // a bad address, a bad value, a bad count, a function we don't have
    CHECK(ask(s, p, {3, 0x03, 0, 15, 0, 2}) == withCrc({3, 0x83, 0x02}));
    CHECK(ask(s, p, {3, 0x05, 0, 40, 0xFF, 0x00}) == withCrc({3, 0x85, 0x02}));
    CHECK(ask(s, p, {3, 0x05, 0, 1, 0x12, 0x34}) == withCrc({3, 0x85, 0x03}));
    CHECK(ask(s, p, {3, 0x03, 0, 0, 0, 0}) == withCrc({3, 0x83, 0x03}));
    CHECK(ask(s, p, {3, 0x10, 0, 0, 0, 2, 3, 1, 2, 3}) == withCrc({3, 0x90, 0x03}));
    CHECK(ask(s, p, {3, 0x2B, 0x0E, 1, 0}) == withCrc({3, 0xAB, 0x01}));

    // a broadcast is done but not answered; another server's request and a bad CRC are left alone
    CHECK(ask(s, p, {0, 0x06, 0, 6, 0x00, 0x07}).empty() && s.getHoldingRegister(6) == 7);
    CHECK(ask(s, p, {4, 0x06, 0, 6, 0x00, 0x08}).empty());
    std::vector<uint8_t> bad = withCrc({3, 0x06, 0, 6, 0x00, 0x09});
    bad.back() ^= 1;
    p.out.clear();
    send(s, p, bad);
    CHECK(p.out.empty() && s.getHoldingRegister(6) == 7);
}

// A transceiver that hears itself
static void echo() {
    ModbusRTUServer s;
    FakePort p;
    s.begin(&p, 115200, 3, -1, true, 16, 16);

    // our reply comes back: not a request
    std::vector<uint8_t> reply = ask(s, p, {3, 0x03, 0, 1, 0, 1});
    CHECK(reply == withCrc({3, 0x03, 2, 0, 0}));
    p.out.clear();
    send(s, p, reply);
    CHECK(p.out.empty() && s.echoDetected());

    // What's held back as a possible echo counts against the buffer's room. Custom framing keeping 500 bytes,
    // most of a reply held, then something else: all of it comes through, in order.
    s.onRawBuffer = [](uint8_t*, size_t) -> size_t { return 0; };
    std::vector<uint8_t> expected(500, 0xAA);
    p.in.insert(p.in.end(), expected.begin(), expected.end());
    s.update();
    uint8_t frame[9] = {3, 0x03, 2, 0x11, 0x22};
    s.sendResponse(frame, 5, false);
    p.in.insert(p.in.end(), frame, frame + 6);
    s.update();
    p.in.push_back(0xEE);
    p.in.insert(p.in.end(), 11, 0xBB);
    expected.insert(expected.end(), frame, frame + 6);
    expected.push_back(0xEE);
    expected.insert(expected.end(), 11, 0xBB);
    std::vector<uint8_t> seen;
    s.onRawBuffer = [&](uint8_t* data, size_t length) -> size_t {
        seen.insert(seen.end(), data, data + length);
        return length;
    };
    s.update();
    CHECK(seen == expected && p.in.empty());
}

// Custom framing: the whole buffer goes to the callback, which keeps what it doesn't take
static void framing() {
    ModbusRTUServer s;
    FakePort p;
    s.begin(&p, 115200, 3, -1, true, 16, 16);

    size_t seen = 0;
    s.onRawBuffer = [&](uint8_t*, size_t length) -> size_t {
        seen = length;
        return length >= 4 ? 4 : 0;
    };
    p.in.insert(p.in.end(), {1, 2, 3, 4, 5, 6});
    s.update();
    CHECK(seen == 6);
    s.update();
    CHECK(seen == 2);

    // more than one read's worth comes in at once
    p.in.insert(p.in.end(), 98, 0x55);
    s.update();
    CHECK(seen == 100);
    s.onRawBuffer = [](uint8_t*, size_t length) -> size_t { return length; };
    s.update();

    // A big frame half in, then more than the buffer holds after a stall. Frames are [length, number, data...]
    // here, and the handler takes whole ones. None may go missing.
    std::vector<int> frames;
    bool intact = true;
    s.onRawBuffer = [&](uint8_t* data, size_t length) -> size_t {
        size_t offset = 0;
        while (offset < length && data[offset] >= 2 && offset + data[offset] <= length) {
            for (size_t i = 2; i < data[offset]; i++)
                intact &= data[offset + i] == data[offset + 1];
            frames.push_back(data[offset + 1]);
            offset += data[offset];
        }
        return offset;
    };
    std::vector<uint8_t> line;
    for (int k = 0; k < 40; k++) {
        uint8_t len = k == 3 ? 200 : 20;
        line.push_back(len);
        line.insert(line.end(), len - 1, (uint8_t) k);
    }
    p.in.insert(p.in.end(), line.begin(), line.begin() + 160);  // 3 frames and the start of the big one
    s.update();
    p.in.insert(p.in.end(), line.begin() + 160, line.end());  // 820 bytes at once
    s.update();
    bool inOrder = frames.size() == 40;
    for (size_t k = 0; k < frames.size(); k++)
        inOrder &= frames[k] == (int) k;
    CHECK(inOrder);
    CHECK(intact);
}

int main() {
    requests();
    echo();
    framing();

    printf("host test: %s\n", fails ? "FAILED" : "ok");
    return fails != 0;
}
