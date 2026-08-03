CXX ?= c++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -g -I..

# Header prerequisites, one variable per header, each pulling in whatever it
# itself includes -- so the transitive closure is written once here rather
# than copied into every rule that needs it. A rule naming only the header a
# test includes DIRECTLY is the bug this replaces: test_uac2_parse listed
# usb_audio2_parse.h but not the usb_audio_parse.h it includes, so editing
# only the shared header left a stale binary that plain `make` reported as up
# to date (it happened to rebuild during the UAC2 work only because the .cpp
# changed in the same commits). If a header gains an include, extend its
# variable and every dependent rule follows.
#
# Compiler-generated deps (-MMD) would remove the hand-maintenance, but not
# with these one-step compile-and-link rules: both translation units derive
# the depfile name from -o, so the second overwrites the first and the test's
# own includes are silently lost. That needs per-object compilation, which is
# a bigger change than this build wants; measured 2026-08-03.
H_PARSE    = ../usb_audio_parse.h
H_PARSE2   = ../usb_audio2_parse.h $(H_PARSE)
H_FEEDBACK = ../usb_audio_feedback.h
H_ISO      = ../ehci_iso.h
H_FIFO     = ../usb_audio_fifo.h

all: run

test_uac1_parse: test_uac1_parse.cpp ../usb_audio_parse.cpp $(H_PARSE)
	$(CXX) $(CXXFLAGS) -o $@ test_uac1_parse.cpp ../usb_audio_parse.cpp

test_sitd: test_sitd.cpp ../ehci_iso.cpp $(H_ISO)
	$(CXX) $(CXXFLAGS) -o $@ test_sitd.cpp ../ehci_iso.cpp

test_itd: test_itd.cpp ../ehci_iso.cpp $(H_ISO)
	$(CXX) $(CXXFLAGS) -o $@ test_itd.cpp ../ehci_iso.cpp

test_feedback: test_feedback.cpp ../usb_audio_feedback.cpp $(H_FEEDBACK)
	$(CXX) $(CXXFLAGS) -o $@ test_feedback.cpp ../usb_audio_feedback.cpp

test_pack: test_pack.cpp ../usb_audio_parse.cpp $(H_PARSE)
	$(CXX) $(CXXFLAGS) -o $@ test_pack.cpp ../usb_audio_parse.cpp

test_uac2_parse: test_uac2_parse.cpp ../usb_audio2_parse.cpp $(H_PARSE2)
	$(CXX) $(CXXFLAGS) -o $@ test_uac2_parse.cpp ../usb_audio2_parse.cpp

test_stream_config: test_stream_config.cpp ../usb_audio_parse.cpp $(H_PARSE)
	$(CXX) $(CXXFLAGS) -o $@ test_stream_config.cpp ../usb_audio_parse.cpp

run: test_uac1_parse test_sitd test_fifo test_feedback test_itd test_pack test_uac2_parse test_stream_config
	./test_uac1_parse
	./test_sitd
	./test_fifo
	./test_feedback
	./test_itd
	./test_pack
	./test_uac2_parse
	./test_stream_config

test_fifo: test_fifo.cpp ../usb_audio_fifo.cpp $(H_FIFO)
	$(CXX) $(CXXFLAGS) -o $@ test_fifo.cpp ../usb_audio_fifo.cpp

clean:
	rm -rf test_fifo test_fifo.dSYM test_uac1_parse test_uac1_parse.dSYM test_sitd test_sitd.dSYM test_feedback test_feedback.dSYM test_itd test_itd.dSYM test_pack test_pack.dSYM test_uac2_parse test_uac2_parse.dSYM test_stream_config test_stream_config.dSYM

.PHONY: all run clean
