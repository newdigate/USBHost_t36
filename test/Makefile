CXX ?= c++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -g -I..

all: run

test_uac1_parse: test_uac1_parse.cpp ../usb_audio_parse.cpp ../usb_audio_parse.h
	$(CXX) $(CXXFLAGS) -o $@ test_uac1_parse.cpp ../usb_audio_parse.cpp

run: test_uac1_parse
	./test_uac1_parse

clean:
	rm -f test_uac1_parse

.PHONY: all run clean
