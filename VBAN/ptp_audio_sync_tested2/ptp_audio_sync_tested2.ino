
#include <Audio.h>
#include <t41-ptp.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include "control_ethernet.h"
#include "input_net.h"
#include "OpenAudio_ArduinoLibrary.h"
#include <QNEthernet.h>
#include "QNEthernetIEEE1588.h"
using namespace qindesign::network;

// Audio config
const float sample_rate_Hz = 48000.0f;
const int audio_block_samples = 128;
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

AudioOutputI2S_F32 audioOutput(audio_settings);
AudioControlEthernet ether1;
AudioInputNet in1(1);
AudioConnection_F32 patchCord1(in1, 0, audioOutput, 0);
AudioConnection_F32 patchCord2(in1, 0, audioOutput, 1);
AudioControlSGTL5000 sgtl;

// PTP config
bool p2p = false;
bool master = false;
bool slave = true;
l3PTP ptp(master, slave, p2p);

// Networking
byte mac[6];
IPAddress staticIP{192, 168, 0, 210};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{192, 168, 0, 6};

// Debug timing
elapsedMillis timerPrint1588;
long timer1 = -3000;

void setup() {
  AudioMemory_F32(50);
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);

  Serial.println("\n\nStarting Multi Audio Stream Test");

  char myHost[] = "Teensy1";
  ether1.setHostName(myHost);
  ether1.begin();
  if (ether1.linkIsUp()) {
    Serial.print("IP: ");
    Serial.println(ether1.getMyIP());
  } else {
    Serial.println("Ethernet is disconnected");
  }

  EthernetIEEE1588.begin();
  Serial.println("IEEE1588 timer initialisé");

  in1.begin();
  sgtl.enable();
  sgtl.volume(1);
  sgtl.unmuteLineout();

  char s1[] = "Stream1dudule";
  in1.subscribe(s1);

  ptp.begin();
  Serial.println("PTP démarré");
}

void loop() {
  // 🧪 Test : injecter un décalage manuel de 10 ms à 20s
  /*
  if (millis() > 20000 && millis() < 20200) {
    Serial.println("----------------------------------------------------------------------");
    Serial.println("[TEST] Décalage manuel via ENET_ATVR += 10ms");
    ENET_ATVR += 10 * 1000 * 1000;

  }
  */

  if (millis() - timer1 > 20000) {
    Serial.printf("---------- Main: %i\n", millis() / 1000);
    Serial.printf("LinkIs Up %i, IP ", ether1.linkIsUp());
    Serial.println(ether1.getMyIP());
    ether1.printHosts();
    //printActiveSubs();

    Serial.println("----------------------------------------------------------------------");
    Serial.println("[TEST] Décalage manuel via ENET_ATVR += 10ms");
    ENET_ATVR += 10 * 1000 * 1000;

    timer1 = millis();
  }

  if (timerPrint1588 > 1000) {
    timespec ts;
    if (EthernetIEEE1588.readTimer(ts)) {
      Serial.printf("[1588] %ld s | %ld ns\n", ts.tv_sec, ts.tv_nsec);
      Serial.printf("[PTP] Offset: %ld ns | Delay: %ld ns\n", ptp.getOffset(), ptp.getDelay());
    } else {
      Serial.println("[1588] Échec lecture timer");
    }
    timerPrint1588 = 0;
  }

  ptp.update();
  delay(100);
}
