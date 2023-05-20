#pragma once
#include <string>
#include <functional>
#include <chrono>

#define ARDUINO_PINS 28

enum pinmode_t { MODE_UNDEFINED, INPUT, INPUT_PULLUP, OUTPUT, SERVO };
enum pincmd_t { CMD_UNDEFINED, DIGITAL_READ, ANALOG_READ, DIGITAL_WRITE, ANALOG_WRITE, SERVO_WRITE };

pinmode_t StringToPinModeEnum( const std::string& enumstring );
pincmd_t StringToPinCmdEnum( const std::string& enumstring );

class XpinConfig
{
public:
	XpinConfig();

	//int pinNo;
	pinmode_t pinMode;
	pincmd_t pinCommand;
	int startval;
	int intval;
	float in_min;
	float in_max;
	float out_min;
	float out_max;
	float in_mid;
	float out_mid;

	int freeze_ms;
	std::chrono::time_point<std::chrono::steady_clock> last_change_time;

	bool IsCommandRefPin();
	bool IsDualCommandRefPin();
	int GetCommandRefIndex(int p_pinread);
	float CalcNewXplaneValue(long int p_pinread);
	bool CalcNewPinValue(float p_refread);

	void ParseInifileMapValue(const std::string& csv_pin_mappings, int pinNo); // map="" under [pinN]

private:

	std::function<float(float)> PinMappedFunction;

	float NoMapPinVal(float x);
	float MapPinVal(float x);
	float MapPinValMid(float x);

	enum pin_map_seq_t { FIRST_MAPPING, MIDDLE_MAPPING, LAST_MAPPING };
	void ParseMapPair( const std::string& str, int pinNo, pin_map_seq_t pin_mapping_sequence, int* p_has_mid_part );  // 1023->1
};



extern std::array<XpinConfig, ARDUINO_PINS> xpins;
