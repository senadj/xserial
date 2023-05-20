#include <thread>
#include "XplaneAsyncSerial.h"
#include "XpinConfig.h"

static std::vector<char> readqueue;

void XplaneAsyncSerial::pingWriteThread()	// ping 4x per second for 2.5s
{
	for (int k = 0; k < 10 ; k++ )
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(250));

		if ( m_ping.load() )    // Arduino init
		{
            for( int i=0; i < xpins.size(); i++ )
            {
                const XpinConfig& thePin = xpins[i];

                if ( thePin.pinMode == OUTPUT ) ArduinoCommand(i,'O');
                else if ( thePin.pinMode == INPUT ) ArduinoCommand(i,'I');
                else if ( thePin.pinMode == INPUT_PULLUP ) ArduinoCommand(i,'P');
                else if ( thePin.pinMode == SERVO ) ArduinoCommand(i,'S');

                if ( thePin.pinCommand == ANALOG_READ ) ArduinoCommand(i,'A');
                else if ( thePin.pinCommand == DIGITAL_READ ) ArduinoCommand(i,'D');
                else if ( thePin.pinCommand == ANALOG_WRITE ) ArduinoCommand(i,'U');
                else if ( thePin.pinCommand == DIGITAL_WRITE ) ArduinoCommand(i,'W');
            }
		}

		ArduinoCommand(255,127); // ping
	}
};

void XplaneAsyncSerial::open(const std::string& devname, unsigned int baud_rate)
{
	m_devname = devname;
	m_baudrate = baud_rate;

	AsyncSerial::open( devname, baud_rate );

	if ( this->isOpen() )
		std::thread([this](){ pingWriteThread(); }).detach();
};

void XplaneAsyncSerial::ArduinoCommand( int pPin, char pCmd )
{
    char buffer[6] = {0,0,0,'0','3','\n'}; // 03 = message length
    snprintf ( buffer, 3, "%02x", pPin ); // this sets [2] with \0
    buffer[2] = pCmd; // but here we overwrite with command
    write( buffer, sizeof(buffer) );
}

void XplaneAsyncSerial::ArduinoPinData(int pin, int val)
{
    char buffer[7] = {0,0,0,0,'0','4','\n'};
    snprintf ( buffer, 3, "%02x", pin );
    snprintf ( &buffer[2], 3, "%02x", val );
    buffer[4] = '0';
    write( buffer, sizeof(buffer) );
}

void XplaneAsyncSerial::ArduinoPinsData( const std::vector<std::pair<int,int>>& vPinVals )
{
	std::vector<char> vchar;
    char buffer[5];
    int msglen = 0;

    for (std::vector<std::pair<int,int>>::const_iterator it = vPinVals.begin(); it!=vPinVals.end(); ++it)
    {
        snprintf ( buffer, 3, "%02x", (*it).first );
        snprintf ( &buffer[2], 3, "%02x", (*it).second );
        vchar.insert( vchar.end(), buffer, buffer+4 );
        msglen += 4;
    }

    if ( msglen > 0 )
    {
        snprintf ( buffer, 3, "%02x", msglen );
        buffer[2] = '\n';
        vchar.insert( vchar.end(), buffer, buffer+3 );
        write( vchar.data(), vchar.size() );
    }
}

XplaneAsyncSerial::XplaneAsyncSerial(): AsyncSerial()
{
    setReadCallback(std::bind(&XplaneAsyncSerial::readCallback, this, std::placeholders::_1, std::placeholders::_2));
    readqueue.reserve(4096);
    vTempIntFloat.reserve(16);
    // vTempIntInt.reserve(16);
    vTempInt.reserve(16);
    m_ping.store(0);
}

void XplaneAsyncSerial::processReadQueue(void)
{
	long int pinNo, pinVal, msglen;
	size_t  slen = readqueue.size()-1; // minus \n
	const char* p = readqueue.data();
	char* pend; // after strtol used for validation from hex to number
	char cbuff2[3]  = {'?','?','\0'}; // used for conversion of hex value to int using strtol
	char cbuff3[4]  = {'?','?','?','\0'}; // used for conversion of hex value to int using strtol

	if ( slen == 0 ) goto cleanup;
	if ( *(p+slen-1) == '\r' ) // arduino serial.println("abc") was used?
	{
		/*logit("serial.println used");*/
		goto cleanup;
	}

	if ( slen < 7 )
		goto cleanup;
	else
		slen -= 2;

	msglen = strtol( p+slen, &pend, 16 );

	if ( msglen%5 != 0 ) goto cleanup;
	if ( static_cast<int>(slen) < msglen ) goto cleanup; // missing beginning of msg?

	if ( static_cast<int>(slen) > msglen )  // serial buffer garbage?
		p += slen - msglen;


	for ( long int i = 0; i < msglen/5; i++ )
	{
		memcpy(&cbuff2[0],p,2);
		pinNo = strtol(cbuff2,&pend,16);
		if (*pend) { /*logit("hex convert error I");*/ goto cleanup; } // hex to num conversion error
		p += 2;

		memcpy(&cbuff3[0],p,3);
		pinVal = strtol(cbuff3,&pend,16);
		if (*pend) { /*logit("hex convert error II");*/ goto cleanup; } // hex to num conversion error
		p += 3;

		if ( pinNo==0xFF && pinVal==0xFFF )
		{
			m_ping.store(1);
			/*logit("Pong returned.");*/
			goto cleanup;
		}

		if ( pinNo >= xpins.size() )
			continue;

        XpinConfig& currPin = xpins[pinNo];

		if ( currPin.intval != pinVal )
		{
            if ( currPin.IsCommandRefPin() )
            {
                int cmdIdx = currPin.GetCommandRefIndex( pinVal );
                if ( cmdIdx >= 0 )
                    vTempInt.push_back( cmdIdx );
            }
            else
            {
                float newval = currPin.CalcNewXplaneValue(pinVal);

                if ( local_dataref_types[pinNo] == 1 )
                    vTempIntInt.emplace_back( pinNo, static_cast<int>( newval ) );
                else
                    vTempIntFloat.emplace_back( pinNo, newval );
            }
		}
	}

	{
		std::lock_guard<std::mutex> guard(m_mutex);

		if ( ! vTempIntFloat.empty() )
            vSetDataf.swap(vTempIntFloat);

		if ( ! vTempIntInt.empty() )
            vSetDatai.swap(vTempIntInt);

		if ( ! vTempInt.empty() )
            vCommandOnce.swap(vTempInt);
	}

	vTempIntFloat.clear();
	vTempIntInt.clear();
	vTempInt.clear();
	readqueue.clear();

	cleanup:
		readqueue.clear();
}

void XplaneAsyncSerial::readCallback(const char *data, size_t len)
{
    readqueue.insert(readqueue.end(),data,data+len);

     if ( data[len-1] == '\n' )
		processReadQueue();
}

void XplaneAsyncSerial::pluginEnable(void)
{
}

void XplaneAsyncSerial::pluginDisable(void)
{
    if ( m_ping.load() )    // Arduino init
    {
        for( int i=0; i < xpins.size(); i++ )
        {
            const XpinConfig& thePin = xpins[i];

            //if ( thePin.pinMode == "OUTPUT" ) {} //maybe change to (default) INPUT ArduinoCommand(i,'I');
            //else if ( thePin.pinMode == "INPUT" ) ArduinoCommand(i,'i');  // same as 'a' + 'd'
            //else if ( thePin.pinMode == "INPUT_PULLUP" ) ArduinoCommand(i,'i');   // same as 'a' + 'd'
            //else if ( thePin.pinMode == "SERVO" ) ArduinoCommand(i,'s');

            if ( thePin.pinCommand == ANALOG_READ ) ArduinoCommand(i,'a');
            else if ( thePin.pinCommand == DIGITAL_READ ) ArduinoCommand(i,'d');
            else if ( thePin.pinCommand == SERVO_WRITE ) ArduinoCommand(i,'s');
            else if ( thePin.pinCommand == ANALOG_WRITE ) ArduinoCommand(i,'u');
        }
    }
	/*m_ping.store(0);*/
}

void XplaneAsyncSerial::pluginStop(void){}

