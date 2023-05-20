BOOST=/ysw/boost_1_82_0
CPPDEPS = -MT$@ -MF`echo $@ | sed -e 's,\.o$$,.d,'` -MD -MP
OBJECTS := AsyncSerial.o XplaneAsyncSerial.o Xutil.o XpinConfig.o plugin.o
CXXFLAGS = -std=c++11 -m64 -O3 -DBOOST_BIND_NO_PLACEHOLDERS -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DAPL=0 -DIBM=0 -DLIN=1 -I$(BOOST)/include -I../SDK/CHeaders/XPLM -fPIC -fvisibility=hidden
LDFLAGS = -static-libgcc -shared -Wl,--version-script=exports.txt -pthread
PROG = xserial.xpl

### Targets: ###
Release: $(PROG)

### Compile: ###
.cpp.o:
	g++ $(CXXFLAGS) -o $@ $(CPPDEPS) -c $<

### Link: ###
$(PROG): $(OBJECTS)
	gcc $(LDFLAGS) -o $(PROG) *.o $(BOOST)/lib/libboost_system.a

### Clean: ###
cleanRelease:
	@find . -name "*.d" -delete; find . -name "*.o" -delete; [ ! -f $(PROG) ] || rm $(PROG)

clean: cleanRelease
### Dependencies tracking: ###
-include ./*.d
