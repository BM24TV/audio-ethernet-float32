#include <QNEthernet.h>
#include "AES67Sender.h"
#include "AudioToRtpL24.h"
#include <Audio.h>

using namespace qindesign::network;

AudioSynthWaveform       wave;       // Source audio : onde sinusoïdale
AES67Sender              sender(IPAddress(239, 69, 22, 1), 5004);
AudioToRtpL24            rtpOut(sender);
AudioConnection          patch(wave, 0, rtpOut, 0);

void setup() {
  Serial.begin(115200);
  AudioMemory(8);
  Ethernet.begin();
  while (!Ethernet.linkStatus()) delay(10);
  sender.begin();

  wave.begin(WAVEFORM_SINE);
  wave.amplitude(0.8);
  wave.frequency(440);  // La note La
}

void loop() {
  // Rien ici, le traitement est dans AudioStream::update()
}
