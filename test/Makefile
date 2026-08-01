CXX ?= c++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -g -I..

all: run

test_uac1_parse: test_uac1_parse.cpp ../usb_audio_parse.cpp ../usb_audio_parse.h
	$(CXX) $(CXXFLAGS) -o $@ test_uac1_parse.cpp ../usb_audio_parse.cpp

test_sitd: test_sitd.cpp ../ehci_iso.cpp ../ehci_iso.h
	$(CXX) $(CXXFLAGS) -o $@ test_sitd.cpp ../ehci_iso.cpp

run: test_uac1_parse test_sitd
	./test_uac1_parse
	./test_sitd

clean:
	rm -rf test_uac1_parse test_uac1_parse.dSYM test_sitd test_sitd.dSYM

.PHONY: all run clean
