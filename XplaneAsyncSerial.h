#pragma once
#include "AsyncSerial.h"
#include "XpinConfig.h"
#include <mutex>

class XplaneAsyncSerial: public AsyncSerial
{
public:
    XplaneAsyncSerial();

    void open(const std::string& devname, unsigned int baud_rate);

	std::string m_devname;
	unsigned int m_baudrate;

	std::atomic<int> m_ping;

	std::mutex m_mutex;	// use this to access m_pinvalmap

    // fill temporary vectors before mutex lock and std::move
    std::vector<std::pair<int,float>> vTempIntFloat;
    std::vector<std::pair<int,int>> vTempIntInt;
    std::vector<int> vTempInt;

    // target vectors for flight loop:
	std::vector<std::pair<int,float>> vSetDataf;
	std::vector<std::pair<int,int>> vSetDatai;
	std::vector<int> vCommandOnce;

	std::array<uint8_t, ARDUINO_PINS> local_dataref_types; // differentiate XPLMGetData integer/float

    void pluginEnable(void);
	void pluginDisable(void);
	void pluginStop(void);

	void ArduinoCommand( int pPin, char pCmd );
	void ArduinoPinData(int pin, int val);
	void ArduinoPinsData( const std::vector<std::pair<int,int>>& vPinVals );

private:

	void pingWriteThread();
    void readCallback(const char *data, size_t len);
    void processReadQueue(void);
};
