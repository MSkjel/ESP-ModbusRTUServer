# The host test, with the sanitizers, as itself and with the reads left out so the writes stand alone. Then the
# library with every function code left out, which has to compile too.
CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror
BUILD ?= build
LIB = ../ModbusRTUServer.cpp ../ModbusRTUServer.h
WRITES_ONLY = -DMODBUS_DISABLE_READ_COILS -DMODBUS_DISABLE_READ_DISCRETE_INPUTS -DMODBUS_DISABLE_REPORT_SERVER_ID
NOTHING = $(addprefix -DMODBUS_DISABLE_,READ_COILS READ_DISCRETE_INPUTS READ_HOLDING_REGISTERS READ_INPUT_REGISTERS \
          REPORT_SERVER_ID WRITE_SINGLE_COIL WRITE_SINGLE_REGISTER WRITE_MULTIPLE_COILS WRITE_MULTIPLE_REGISTERS)

check: $(BUILD)/host_test $(BUILD)/host_test_writes
	$(BUILD)/host_test
	$(BUILD)/host_test_writes
	$(CXX) $(CXXFLAGS) -I.. -fsyntax-only $(NOTHING) ../ModbusRTUServer.cpp

$(BUILD)/host_test: host_test.cpp $(LIB)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I.. -fsanitize=address,undefined -o $@ host_test.cpp ../ModbusRTUServer.cpp

$(BUILD)/host_test_writes: host_test.cpp $(LIB)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I.. -fsanitize=address,undefined $(WRITES_ONLY) -o $@ host_test.cpp ../ModbusRTUServer.cpp

clean:
	rm -rf $(BUILD)

.PHONY: check clean
