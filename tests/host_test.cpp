// The library on a PC, without Arduino or ESP-IDF: requests in through a fake Port, answers out.
//   g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -I.. host_test.cpp ../ModbusRTUServer.cpp -o host_test && ./host_test
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
    size_t read(uint8_t* data, size_t max) override {
        size_t n = 0;
        for (; n < max && !in.empty(); n++) {
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

int main() {
    ModbusRTUServer s;
    FakePort p;
    s.begin(&p, 115200, 3, -1, true, 16, 16);
    s.setHoldingRegister(1, 0x1234);
    s.setHoldingRegister(2, 0xABCD);

    // read holding registers 1-2
    send(s, p, withCrc({3, 0x03, 0, 1, 0, 2}));
    CHECK(p.out == withCrc({3, 0x03, 4, 0x12, 0x34, 0xAB, 0xCD}));

    // its echo, as a transceiver that hears itself gives it back: not a request
    std::vector<uint8_t> echo = p.out;
    p.out.clear();
    send(s, p, echo);
    CHECK(p.out.empty() && s.echoDetected());

    // write register 5, answered with the request
    send(s, p, withCrc({3, 0x06, 0, 5, 0x00, 0x2A}));
    CHECK(s.getHoldingRegister(5) == 0x2A);
    CHECK(p.out == withCrc({3, 0x06, 0, 5, 0x00, 0x2A}));
    p.out.clear();

    // a broadcast is done but not answered; another server's request and a bad CRC are left alone
    send(s, p, withCrc({0, 0x06, 0, 6, 0x00, 0x07}));
    CHECK(s.getHoldingRegister(6) == 7 && p.out.empty());
    send(s, p, withCrc({4, 0x06, 0, 6, 0x00, 0x08}));
    std::vector<uint8_t> bad = withCrc({3, 0x06, 0, 6, 0x00, 0x09});
    bad.back() ^= 1;
    send(s, p, bad);
    CHECK(s.getHoldingRegister(6) == 7 && p.out.empty());

    // custom framing: the whole buffer goes to the callback, which keeps what it doesn't take
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

    // A big frame half in, then more than the buffer holds after a stall. Frames are [length, number, data...]
    // here, and the handler takes whole ones. None may go missing.
    s.onRawBuffer = [](uint8_t*, size_t length) -> size_t { return length; };  // what's left from above
    s.update();
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
    CHECK(inOrder && intact);

    printf("host test: %s\n", fails ? "FAILED" : "ok");
    return fails != 0;
}
