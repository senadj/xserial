#pragma once
#include <boost/property_tree/ptree.hpp>
#include <map>
#include <vector>
#include "XpinConfig.h"

typedef std::vector<std::pair<std::string,int>> VecStrIntType;

class Xutil
{
public:
    Xutil() = default;

    void processIniFile(const char* p_xserial_ini,
                        uint8_t* dataref_types_data,
                        int* p_nth_frame_mask,
                        VecStrIntType& pin_to_dataref,
                        VecStrIntType& dataref_to_pin,
                        VecStrIntType& override_joystick);

    std::string GetPort();
    std::string GetBaudrate();

private:

    std::string m_inifile;
    boost::property_tree::ptree m_tree;

    uint8_t* m_dref_types; // local, so I don't pass it around much in functions

    std::pair<std::string,int> ParseDataRefPinKeyVal ( const std::string& keydata, const std::string& valdata );
    pincmd_t ParseInifilePinCmdVal( const std::string& valdata, int pinNo );
    int GetNthFrameMask( int min_target_frames );
    int IsPinIniEntry( const std::string& str );

};
