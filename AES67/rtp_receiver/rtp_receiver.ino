#include <QNEthernet.h>
#include <QNEthernetIEEE1588.h>
#include <OpenAudio_ArduinoLibrary.h>
#include <Audio.h>
#include <string>
using namespace qindesign::network;

// === Configuration réseau ===
IPAddress localIP(192, 168, 1, 71);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// === Multicast AES67 ===
IPAddress multicastIP(224, 0, 1, 129);
constexpr uint16_t port = 5005;

// === UDP et RTP ===
EthernetUDP udp;
constexpr size_t MAX_PACKET_SIZE = 1500;
uint8_t packetBuffer[MAX_PACKET_SIZE];
constexpr int RTP_HEADER_SIZE = 12;

// === Audio Float32 ===
AudioOutputI2S_F32      i2s_out;
AudioPlayQueue_F32      queue_f32;
AudioControlSGTL5000    sgtl5000;
AudioSynthWaveformSine  dummySine;
AudioConnection_F32     patchCord1(queue_f32, 0, i2s_out, 0);
AudioConnection_F32     patchCord2(queue_f32, 0, i2s_out, 1);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("🔊 Démarrage du récepteur RTP L24...");

  Ethernet.setHostname("rtp-l24-receiver");
  Ethernet.begin(localIP, subnet, gateway);
  Ethernet.setDnsServerIP(dns);
  while (Ethernet.linkStatus() != LinkON) {
    Serial.println("⏳ En attente du lien Ethernet...");
    delay(500);
  }

  Serial.print("IP locale : ");
  Serial.println(Ethernet.localIP());

  udp.beginMulticast(multicastIP, port);
  Serial.println("✅ En écoute sur 224.0.1.129:5005 (RTP L24)");

  AudioMemory(20);
  AudioMemory_F32(24);  // float32
  sgtl5000.enable();
  sgtl5000.volume(0.7);
  dummySine.frequency(0);

  Serial.println("✅ Initialisation audio terminée.");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize > (int)RTP_HEADER_SIZE && packetSize < (int)MAX_PACKET_SIZE) {
    int len = udp.read(packetBuffer, packetSize);
    if (len <= RTP_HEADER_SIZE) return;

    uint8_t* payload = packetBuffer + RTP_HEADER_SIZE;
    int payloadSize = len - RTP_HEADER_SIZE;

    if (payloadSize % 3 != 0) return;  // L24 = 3 octets par sample
    int sampleCount = payloadSize / 3;
    int samplesConsumed = 0;

    int blocksPlayed = 0;

    while (sampleCount - samplesConsumed >= AUDIO_BLOCK_SAMPLES) {
      float32_t* buffer = queue_f32.getBuffer();
      if (buffer != nullptr) {
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
          uint8_t b1 = payload[(samplesConsumed + i) * 3 + 0];
          uint8_t b2 = payload[(samplesConsumed + i) * 3 + 1];
          uint8_t b3 = payload[(samplesConsumed + i) * 3 + 2];

          int32_t sample = (b1 << 16) | (b2 << 8) | b3;
          if (sample & 0x800000) sample |= 0xFF000000;

          buffer[i] = sample / 8388608.0f;
        }

        queue_f32.playBuffer();
        samplesConsumed += AUDIO_BLOCK_SAMPLES;
        blocksPlayed++;
      } else {
        break;
      }
    }

    // Affichage uniquement si des blocs ont été lus
    if (blocksPlayed > 0) {
      Serial.printf("🔁 %d blocs lus (%d échantillons)\n", blocksPlayed, blocksPlayed * AUDIO_BLOCK_SAMPLES);
    }

  } else {
    static unsigned long waitPrint = 0;
    if (millis() - waitPrint > 1000) {
      Serial.println("⏳ En attente de paquets RTP...");
      waitPrint = millis();
    }
  }
}
