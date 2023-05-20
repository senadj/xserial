#include "Xutil.h"
#include <boost/property_tree/ini_parser.hpp>
#include <cmath>

std::string Xutil::GetPort() { return m_tree.get<std::string>("port", "/dev/ttyACM0"); }
std::string Xutil::GetBaudrate() { return m_tree.get<std::string>("baudrate", "115200"); }

pincmd_t Xutil::ParseInifilePinCmdVal( const std::string& valdata, int pinNo )  // analogWrite(45)
{
    std::string initial_cmd_val_str, cmd_str = valdata;
    std::size_t found1, found2;
    found1 = valdata.find("(");
    found2 = valdata.find(")");

    if ( found1 != std::string::npos )
    {
        cmd_str = valdata.substr(0,found1);

        if ( found2 != std::string::npos )
        {
            initial_cmd_val_str = valdata.substr(found1+1, found2-found1-1);
            if ( ! initial_cmd_val_str.empty() )
            {
                try { xpins[pinNo].startval = std::stoi(initial_cmd_val_str); }
                catch ( std::invalid_argument& ) {}
            }
        }
    }

    return StringToPinCmdEnum(cmd_str);
}

int Xutil::GetNthFrameMask( int min_target_frames )
{
    int i = ceil(log(min_target_frames)/log(2)); // extract: https://stackoverflow.com/a/466256
    return (1 << i) - 1;   // https://stackoverflow.com/a/19760152
}

int Xutil::IsPinIniEntry( const std::string& str )
{
	if ( str.rfind("pin", 0) == 0 )
	{
		char* endptr;
		const char* ptr = str.substr(3).c_str();
		long pinNo = strtol(ptr,&endptr,10);
		if ( endptr == ptr + strlen(ptr) )
			return pinNo;
	}
	return -1;
}

std::pair<std::string,int> Xutil::ParseDataRefPinKeyVal ( const std::string& keydata, const std::string& valdata )
{
    int numval, is_pin;
    std::string keypart1, keypart2 = keydata;   // we presume keypart1 = "float" (=default);

    if ( valdata.rfind("pin", 0) == 0 )
    {
        numval = std::stoi(valdata.substr(3));
        is_pin = 1;
    }
    else
    {
        numval = std::stoi(valdata);
        is_pin = 0;
    }

    if ( keydata.rfind("(", 0) == 0 )
    {
        std::size_t found = keydata.find(")");
        if (found != std::string::npos)
        {
            keypart1 = keydata.substr(1,found-1);
            keypart2 = keydata.substr(found+1);
            if (is_pin)
            {
                if ( keypart1 == "int" && numval < xpins.size() )
                    m_dref_types[numval] = 1;   // setting array values from plugin.cpp
            }
        }
    }

    return std::make_pair( keypart2, numval );
}

void Xutil::processIniFile( const char* p_xserial_ini,
                            uint8_t* dataref_types_data,
                            int* p_nth_frame_mask,
                            VecStrIntType& pin_to_dataref,
                            VecStrIntType& dataref_to_pin,
                            VecStrIntType& override_joystick)
{
    m_inifile = std::string(p_xserial_ini);
    m_dref_types = dataref_types_data;

	try
	{
		boost::property_tree::ini_parser::read_ini( m_inifile, m_tree );

		*p_nth_frame_mask = GetNthFrameMask( m_tree.get<int>("write_every_Nth_frame", 32 /* default or bad data */ ) );

		for (const auto& it: m_tree) // std::pair<key_type, property_tree>
		{
			if ( ! it.second.empty() ) // it's a parent node for example: [demo]
			{
				const std::string& pkey = it.first.data();
				const boost::property_tree::ptree & subtree = it.second;
				/*for (auto& ch: subtree)
						kvmap[pkey+'.'+ch.first.data()] = ch.second.data();*/

                if ( pkey == "XPLMFlightLoop_f/XPLMSetData<-in" )
                    for (auto& ch: subtree)
                        pin_to_dataref.push_back( ParseDataRefPinKeyVal( ch.first.data(), ch.second.data() ) );

                else if ( pkey == "XPLMFlightLoop_f/XPLMGetData->out" )
                    for (auto& ch: subtree)
                        dataref_to_pin.push_back( ParseDataRefPinKeyVal( ch.first.data(), ch.second.data() ) );

                else if ( pkey == "XPLM_MSG_PLANE_LOADED/XPLMSetData<-in" )
                {
                    for (auto& ch: subtree)
                        override_joystick.push_back(std::make_pair(ch.first.data(), std::stoi(ch.second.data())));
                }
                else
                {
                    int pinNo = IsPinIniEntry(pkey); // if *.ini entry [pinXX] starts with "pin" returns "XX" else -1

                    if ( pinNo >= 0 )
                    {
                        for (const auto& ch: subtree)
                        {
                            const std::string& ini_ch_key = ch.first.data();
                            const std::string& ini_ch_val = ch.second.data();
                            XpinConfig& targetPin = xpins[pinNo];

                            if (ini_ch_key == "mode")                                 // example: "mode=INPUT"
                                targetPin.pinMode = StringToPinModeEnum( ini_ch_val );
                            else if (ini_ch_key == "command")                         // example: "command=digitalWrite(1)"
                                targetPin.pinCommand = ParseInifilePinCmdVal( ini_ch_val, pinNo );
                            else if (ini_ch_key == "map")                             // example: "map=0->-1,1023->1"
                                targetPin.ParseInifileMapValue( ini_ch_val, pinNo );
                            else if (ini_ch_key == "freeze_ms")                             // freeze changes after pin val modified
                                targetPin.freeze_ms = std::stoi( ini_ch_val );
                        }
                    }
                }
			}
			else // root key-val pairs:
			{
                //kvmap[it.first.data()] = it.second.data();
			}
		}

		//tree.put("debug.filename", "dfile1"); // overwrites value on each run
		//tree.put("debug.level", "dlevel1");
		//boost::property_tree::write_ini( inifile, tree);
	}
	catch (boost::property_tree::ptree_error &e)
	{
		//std::cout << "ini file error:\n" << e.what() << std::endl;
	}
}
