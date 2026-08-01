#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
USBAudioOut audioOut(myusb);

void setup() {
	while (!Serial && millis() < 3000) ;
	Serial.println("UAC1 enumeration test");
	audioOut.format(48000, 2, 16);
	myusb.begin();
}

bool reported = false;

void loop() {
	myusb.Task();
	if (audioOut.ready() && !reported) {
		const UAC1Topology &t = audioOut.topology();
		Serial.printf("UAC %x.%02x  control if=%d  streaming if=%d  feature unit=%d\n",
			t.bcd_adc >> 8, t.bcd_adc & 0xFF, t.control_interface,
			t.streaming_interface, t.feature_unit_id);
		for (uint8_t i = 0; i < t.alt_count; i++) {
			const UAC1AltSetting &a = t.alts[i];
			Serial.printf("  alt %d: ep=0x%02X attr=0x%02X mps=%d ch=%d bits=%d rate=%lu\n",
				a.alternate_setting, a.endpoint_address, a.endpoint_attributes,
				a.max_packet_size, a.channels, a.bit_resolution,
				(unsigned long)a.sample_rate);
		}
		Serial.printf("selected alternate setting %d\n", audioOut.alternateSetting());
		reported = true;
	}
	if (!audioOut.ready()) reported = false;
}
