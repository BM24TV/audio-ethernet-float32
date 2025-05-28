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

IPAddress ip(192, 168, 2, 210);
IPAddress gateway(192, 168, 2, 254);
IPAddress subnet(255, 255, 255, 0);
Ethernet.begin(ip, gateway, subnet);

  while (!Ethernet.linkStatus()) {
    Serial.println("En attente du lien réseau...");
    delay(100);
  }



  sender.begin();

  

  wave.begin(WAVEFORM_SINE);
  wave.amplitude(0.8);
  wave.frequency(440);  // La note La

Serial.println(Ethernet.localIP());


}

void loop() {
  // Rien ici, le traitement est dans AudioStream::update()
}
