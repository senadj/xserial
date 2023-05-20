#include "XpinConfig.h"
#include <boost/algorithm/string.hpp>
#include <vector>
#include<cfloat> // for FLT_MAX
#include <map>

std::array<XpinConfig,ARDUINO_PINS> xpins;
std::map<std::string,int> XPLMCommandToIndexMap;

XpinConfig::XpinConfig() :  pinMode(MODE_UNDEFINED), pinCommand(CMD_UNDEFINED),
                            startval(INT_MIN), intval(INT_MIN), // std. header <limits.h>
                            in_min(-FLT_MAX), in_max(FLT_MAX),
                            out_min(-FLT_MAX), out_max(FLT_MAX),
                            in_mid(0.0f), out_mid(0.0f),
                            freeze_ms(0), last_change_time(std::chrono::steady_clock::now())
                                        //timepoint == std::chrono::milliseconds::zero()
{
    PinMappedFunction = std::bind(&XpinConfig::NoMapPinVal, this, std::placeholders::_1);
};



pinmode_t StringToPinModeEnum( const std::string& enumstring )
{
    const std::string upperstr = boost::algorithm::to_upper_copy(enumstring);
    if ( upperstr == "INPUT" ) return INPUT;
    else if ( upperstr == "INPUT_PULLUP" ) return INPUT_PULLUP;
    else if ( upperstr == "OUTPUT" ) return OUTPUT;
    else if ( upperstr == "SERVO" ) return SERVO;
    else return MODE_UNDEFINED;
}

pincmd_t StringToPinCmdEnum( const std::string& enumstring )
{
    const std::string lowerstr = boost::algorithm::to_lower_copy(enumstring);
    if ( lowerstr == "digitalread" ) return DIGITAL_READ;
    else if ( lowerstr == "analogread" ) return ANALOG_READ;
    else if ( lowerstr == "digitalwrite" ) return DIGITAL_WRITE;
    else if ( lowerstr == "analogwrite" ) return ANALOG_WRITE;
    else if ( lowerstr == "servo.write" ) return SERVO_WRITE;
    else return CMD_UNDEFINED;
}


bool XpinConfig::IsCommandRefPin() { return ( in_mid == FLT_MAX ); }

bool XpinConfig::IsDualCommandRefPin() { return ( out_mid == FLT_MAX ); }

int XpinConfig::GetCommandRefIndex(int p_pinread)
{
    auto now = std::chrono::steady_clock::now();
    if ( freeze_ms > std::chrono::duration_cast<std::chrono::milliseconds>(now - last_change_time).count() )
        return -1;

    last_change_time = now;
    intval = p_pinread;

    if ( p_pinread == static_cast<int>(in_min) )
        return static_cast<int>(out_min);

    else if ( p_pinread == static_cast<int>(in_max) )
        return static_cast<int>(out_max);

    return -1;
}



float XpinConfig::CalcNewXplaneValue(long int p_pinread)    // called from serial read function
{
    intval = p_pinread;
    return PinMappedFunction( static_cast<float>(p_pinread) ); // PinMapFunction( static_cast<float>(p_pinread) );
}

bool XpinConfig::CalcNewPinValue(float p_refread)   // called from flight loop
{
    int calculated_val = static_cast<int>( PinMappedFunction( p_refread ) );

    if ( calculated_val == intval )
        return false;
    else
        intval = calculated_val;

    return true;
}


float XpinConfig::NoMapPinVal(float x)
{
    return x;
}


float XpinConfig::MapPinVal(float x)
{
    float r;
    if ( x > in_max )
        r = in_max;
    else if ( x < in_min )
        r = in_min;
    else
        r = x;

    return (r - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float XpinConfig::MapPinValMid(float x)
{
    float r;
    if ( x > in_max )
        r = in_max;
    else if ( x < in_min )
        r = in_min;
    else
        r = x;

    if ( r >= in_mid )
        return (r - in_mid) * (out_max - out_mid) / (in_max - in_mid) + out_mid;
    else
        return (r - in_min) * (out_mid - out_min) / (in_mid - in_min) + out_min;
}



void XpinConfig::ParseMapPair( const std::string& str, int pinNo, pin_map_seq_t pin_mapping_sequence, int* p_has_mid_part)
{
    if ( str.empty() ) return;
    float floatLow, floatHigh;
    std::string stringII, stringI = str;   // only part of middle data, without '->XX'
    std::size_t found = str.find("->");

    if ( found != std::string::npos )
    {
        stringI = str.substr(0,found);
        stringII = str.substr(found+2);
    }

    try { floatLow = std::stof(stringI); }	catch ( std::invalid_argument& ) { /*is_ok = false;*/ }
    try { floatHigh = std::stof(stringII); }	catch ( std::invalid_argument& )
    {
        // can not convert to float
        if ( ! stringII.empty() && stringII.find("/") != std::string::npos )  // possibly XPLMCommand
        {                // here we abuse member variables with alt meaning:
            if ( pin_mapping_sequence == FIRST_MAPPING )
            {
                in_mid = FLT_MAX;   // flag it as CommandRef pin (example: button=LOW -> cmd)
                out_min = static_cast<float>(pinNo); // store 'min' CommandRef index -> cmd refs[idx]
                XPLMCommandToIndexMap[stringII] = pinNo;
            }
            else if ( pin_mapping_sequence == LAST_MAPPING )
            {
                out_mid = FLT_MAX;  // flag it as CommandRef pin with two values (example 0=gear down, 1=gear up)
                out_max = static_cast<float>(pinNo + ARDUINO_PINS); // store 'max' CommandRef index -> cmd refs[idx]
                XPLMCommandToIndexMap[stringII] = pinNo + ARDUINO_PINS;
            }
        }
    }

    if ( pin_mapping_sequence == FIRST_MAPPING )
    {
        in_min = floatLow;
        if ( ! IsCommandRefPin() )  out_min = floatHigh;
    }
    else if ( pin_mapping_sequence == LAST_MAPPING )
    {
        in_max = floatLow;
        if ( ! IsDualCommandRefPin() )  out_max = floatHigh;
    }
    else    // MIDDLE_MAPPING
    {
        if ( ! IsCommandRefPin() ) in_mid = floatLow;
        (*p_has_mid_part)++;
    }
}


void XpinConfig::ParseInifileMapValue(const std::string& csv_pin_mappings, int pinNo)   // // 0->-1,1023->1
{
    std::vector<std::string> vpin_mappings;
    boost::split(vpin_mappings, csv_pin_mappings, boost::is_from_range(',',',') );
    size_t  cnt = 0;
    int has_mid_part = 0;

    for ( const auto& pin_mapping: vpin_mappings )
    {
        cnt++;
        if ( cnt == 1 )	// first piece "0->-1"
            ParseMapPair( pin_mapping, pinNo, FIRST_MAPPING, &has_mid_part );
        else if ( cnt == vpin_mappings.size() )	// last piece "1023->"
            ParseMapPair( pin_mapping, pinNo, LAST_MAPPING, &has_mid_part );
        else 								// mid piece  "550" or "550->"
            ParseMapPair( pin_mapping, pinNo, MIDDLE_MAPPING, &has_mid_part );
    }

    if ( this->IsCommandRefPin() )
        return;

    if ( has_mid_part > 0 )
    {
        out_mid = (out_min + out_max) / 2;
        PinMappedFunction = std::bind(&XpinConfig::MapPinValMid, this, std::placeholders::_1);
    }
    else
    {
        PinMappedFunction = std::bind(&XpinConfig::MapPinVal, this, std::placeholders::_1);
    }
}

