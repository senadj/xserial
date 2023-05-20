#include <cstring>
#include <cstdio>
#include <XPLMProcessing.h>
#include <XPLMDataAccess.h>
#include <XPLMUtilities.h>
#include <XPLMPlugin.h>
#include <XPLMPlanes.h>
#include "XplaneAsyncSerial.h"
#include "Xutil.h"

extern std::map<std::string,int> XPLMCommandToIndexMap; // defined and populated in XpinConfig.cpp

// use array for max performance. some fields will be unused:
std::array<XPLMCommandRef, 2 * ARDUINO_PINS> cmdrefs;   // max two commands mapped per pin (HIGH/LOW)
std::array<XPLMDataRef, ARDUINO_PINS> datarefs;
std::array<uint8_t, ARDUINO_PINS> dataref_types;   // help differentiate XPLMGet/SetData integer/float in flight loop

char* p_xserial_log;
char* p_xserial_ini;
char* p_xdo_plane;
FILE* xlogfile;
Xutil xutil;
XplaneAsyncSerial xserial;
int mutex_lock_miss_count; // diagnostics
int frame_counter;
int nth_frame_mask;

VecStrIntType override_joystick, pin_to_dataref, dataref_to_pin; // Xutil.h: std::vector<std::pair<std::string,int>>


static void Xlogit(const char* data)
{
	fprintf(xlogfile, "%s\n", data);
	fflush(xlogfile);
}
static void Xlogit(const std::string& data)
{
	fprintf(xlogfile, "%s\n", data.c_str());
	fflush(xlogfile);
}

static float MyFlightLoopCallback( float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon )
{
    if ( xserial.m_mutex.try_lock() )
    {
        std::lock_guard<std::mutex> guard(xserial.m_mutex, std::adopt_lock);

        if ( ! xserial.vSetDataf.empty() )
        {
            for ( const auto& it: xserial.vSetDataf )
                XPLMSetDataf( datarefs[it.first], it.second );

            xserial.vSetDataf.clear();
        }
        if ( ! xserial.vSetDatai.empty() )
        {
            for ( const auto& it: xserial.vSetDatai )
                XPLMSetDatai( datarefs[it.first], it.second );

            xserial.vSetDatai.clear();
        }
        if ( ! xserial.vCommandOnce.empty() )
        {
            for ( const auto& it: xserial.vCommandOnce )
                XPLMCommandOnce( cmdrefs[it] );

            xserial.vCommandOnce.clear();
        }
    }
    else
        mutex_lock_miss_count++;

    frame_counter++;
    if ( (frame_counter & nth_frame_mask) == 0 )    // serial write every Nth frames (ini file)
    {
        std::vector<std::pair<int,int>> vpinvals;
        vpinvals.reserve( dataref_to_pin.size() );

        for ( const auto& it: dataref_to_pin )
        {
            XpinConfig& xPin = xpins[it.second];

            if ( dataref_types[it.second] == 1 )
            {
                int xint = XPLMGetDatai( datarefs[it.second] );

                if ( xPin.CalcNewPinValue( static_cast<float>(xint)) )
                    vpinvals.emplace_back( it.second, xPin.intval );
            }
            else
            {
                float xfloat = XPLMGetDataf( datarefs[it.second] );

                if ( xPin.CalcNewPinValue( xfloat) )
                    vpinvals.emplace_back( it.second, xPin.intval );
            }
        }

        if ( ! vpinvals.empty() )
        {
            xserial.ArduinoPinsData(vpinvals);  // serial write
            vpinvals.clear();
        }
    }

	return -1.0f; /* every frame */
}

PLUGIN_API int XPluginStart( char* outName, char* outSig, char* outDesc )
{
	strcpy(outName, "xserial");
	strcpy(outSig, "xp.uart.xserial");
	strcpy(outDesc, "X-Plane control over serial communication.");

	char xplane_dir[512];
    XPLMGetSystemPath(xplane_dir);
    asprintf( &p_xserial_log, "%s%s", xplane_dir, "Resources/plugins/xserial.log" );
    asprintf( &p_xserial_ini, "%s%s", xplane_dir, "Resources/plugins/xserial.ini" );
    asprintf( &p_xdo_plane, "%s%s", xplane_dir, "xdo-plane" );

	xlogfile = fopen(p_xserial_log, "w");

	xutil.processIniFile(   p_xserial_ini,
                            dataref_types.data(),
                            &nth_frame_mask,
                            pin_to_dataref,
                            dataref_to_pin,
                            override_joystick);

    xserial.local_dataref_types = dataref_types; // create local copy to avoid shared access

    for ( const auto& it: XPLMCommandToIndexMap )
    {
        XPLMCommandRef cmdref = XPLMFindCommand( it.first.c_str() );
        if ( cmdref == NULL )
            Xlogit( "XPLMCommand NOT found: " + it.first );
        else
            cmdrefs[it.second] = cmdref;
    }


    for ( const auto& it: pin_to_dataref )
        datarefs[it.second] = XPLMFindDataRef((it.first).c_str());

    for ( const auto& it: dataref_to_pin )
        datarefs[it.second] = XPLMFindDataRef((it.first).c_str());

    try
    {
        Xlogit( "Opening serial port " + xutil.GetPort() + " ( " + xutil.GetBaudrate() + " bits/s )");
		xserial.open( xutil.GetPort(), std::stoi( xutil.GetBaudrate() ) ); // "/dev/ttyUSB0", 115200
		Xlogit("... connected.");
    }
    catch( boost::system::system_error& e )
    {
        Xlogit( e.what() );
        //return 1;
    }

	XPLMRegisterFlightLoopCallback( MyFlightLoopCallback, 1.0 /* Interval */, NULL /* refcon */);
	return 1;
}

PLUGIN_API void	XPluginStop(void)
{
    XPLMUnregisterFlightLoopCallback(MyFlightLoopCallback, NULL);

	fprintf( xlogfile, "mutex lock miss count: %ul\n", mutex_lock_miss_count );
    Xlogit( "Stopped." );
    xserial.close();

	fclose(xlogfile);
    if (p_xdo_plane) free(p_xdo_plane);
    if (p_xserial_log) free(p_xserial_log);
    if (p_xserial_log) free(p_xserial_ini);
}

PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    xserial.pluginDisable();
}

PLUGIN_API void XPluginReceiveMessage( XPLMPluginID	inFromWho, int inMessage, void* inParam )
{
    if ( ! (inFromWho == XPLM_PLUGIN_XPLANE && inParam == 0) )
        return;

	if ( inMessage == XPLM_MSG_PLANE_LOADED )
	{
        if ( xserial.m_ping.load() )
            Xlogit("Ping back from serial OK.");
        else
            Xlogit("waiting ping back from serial ...");

		char file[512], path[512];
		XPLMGetNthAircraftModel(0, file, path);
		Xlogit( file );
		//if (strcmp(file, "Cessna_172SP.acf") == 0) {}

		for (const auto& it: override_joystick)
            XPLMSetDatai(XPLMFindDataRef((it.first).c_str()), it.second);
	}
}
