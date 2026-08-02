CXX ?= c++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -g -I..

all: run

test_uac1_parse: test_uac1_parse.cpp ../usb_audio_parse.cpp ../usb_audio_parse.h
	$(CXX) $(CXXFLAGS) -o $@ test_uac1_parse.cpp ../usb_audio_parse.cpp

test_sitd: test_sitd.cpp ../ehci_iso.cpp ../ehci_iso.h
	$(CXX) $(CXXFLAGS) -o $@ test_sitd.cpp ../ehci_iso.cpp

test_itd: test_itd.cpp ../ehci_iso.cpp ../ehci_iso.h
	$(CXX) $(CXXFLAGS) -o $@ test_itd.cpp ../ehci_iso.cpp

test_feedback: test_feedback.cpp ../usb_audio_feedback.cpp ../usb_audio_feedback.h
	$(CXX) $(CXXFLAGS) -o $@ test_feedback.cpp ../usb_audio_feedback.cpp

test_pack: test_pack.cpp ../usb_audio_parse.cpp ../usb_audio_parse.h
	$(CXX) $(CXXFLAGS) -o $@ test_pack.cpp ../usb_audio_parse.cpp

run: test_uac1_parse test_sitd test_fifo test_feedback test_itd test_pack
	./test_uac1_parse
	./test_sitd
	./test_fifo
	./test_feedback
	./test_itd

test_fifo: test_fifo.cpp ../usb_audio_fifo.cpp ../usb_audio_fifo.h
	$(CXX) $(CXXFLAGS) -o $@ test_fifo.cpp ../usb_audio_fifo.cpp

clean:
	rm -rf test_fifo test_fifo.dSYM test_uac1_parse test_uac1_parse.dSYM test_sitd test_sitd.dSYM test_feedback test_feedback.dSYM test_itd test_itd.dSYM test_pack test_pack.dSYM

.PHONY: all run clean
