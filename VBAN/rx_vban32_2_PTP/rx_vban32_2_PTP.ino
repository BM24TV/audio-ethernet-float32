#include <Audio.h>

#ifdef HAVE_AUDIO_BOARD
  #include <Wire.h>
  #include <SPI.h>
  #include <SD.h>
  #include <SerialFlash.h>
#endif

#include "control_ethernet.h"
#include "input_net.h"
#include "OpenAudio_ArduinoLibrary.h"
#include <QNEthernet.h>
#include "QNEthernetIEEE1588.h"
using namespace qindesign::network;

EthernetUDP ptpUdp;

// Timer IEEE1588
elapsedMillis timerPrint1588;

void checkPTP() {
  int packetSize = ptpUdp.parsePacket();
  if (packetSize > 0) {
    IPAddress ip = ptpUdp.remoteIP();
    Serial.printf("[PTP] Paquet reçu (%d octets) depuis %d.%d.%d.%d:%d\n",
                  packetSize,
                  ip[0], ip[1], ip[2], ip[3],
                  ptpUdp.remotePort());

    uint8_t buffer[1500];
    ptpUdp.read(buffer, packetSize);

    uint8_t messageType = buffer[0]; // Premier octet du payload = type PTP

    const char* ptpTypeName = "?";
    switch (messageType) {
      case 0x00: ptpTypeName = "SYNC"; break;
      case 0x01: ptpTypeName = "DELAY_REQ"; break;
      case 0x08: ptpTypeName = "FOLLOW_UP"; break;
      case 0x09: ptpTypeName = "DELAY_RESP"; break;
    }

    Serial.printf("[PTP] Type: 0x%02X (%s)\n", messageType, ptpTypeName);

  /*
  if (messageType == 0x00) { // SYNC
    uint32_t ptp_now = ENET_ATVR;
    Serial.printf("[PTP] SYNC reçu — Timer PTP: %u ns\n", ptp_now);
    }
    */



  }
}


const float sample_rate_Hz = 48000.0f;
const int   audio_block_samples = 128;  // Always 128
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

AudioOutputI2S_F32 audioOutput(audio_settings);
AudioControlEthernet ether1;
AudioInputNet in1(1);

//create audio connections
AudioConnection_F32 patchCord1(in1, 0, audioOutput, 0);
AudioConnection_F32 patchCord2(in1, 0, audioOutput, 1);
AudioControlSGTL5000 sgtl;

#include "utils.h"

void setup() {
  AudioMemory_F32(50);

  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  Serial.println("\n\nStarting Multi Audio Stream Test");

  char myHost[] = "Teensy1";
  ether1.setHostName(myHost);
  ether1.begin();
  if (!ether1.linkIsUp())
    Serial.printf("Ethernet is disconnected");
  else
    Serial.println(ether1.getMyIP());

    // Initialisation IEEE1588
  EthernetIEEE1588.begin();
  Serial.println("IEEE1588 timer initialisé");  


  in1.begin(); 
  sgtl.enable();
  sgtl.volume(1);
  sgtl.unmuteLineout();

  char s1[] = "Stream1dudule";
  in1.subscribe(s1);

  // Démarre l'écoute des trames PTP (port 319)
  ptpUdp.begin(319);
  ptpUdp.beginMulticast(IPAddress(224, 0, 1, 129), 319);  // 🔥 Important pour recevoir le multicast PTP

  Serial.println("Done setup");
}

#define EVERY 1000
long count = 0;
long timer1 = -3000;

void loop() {
  if (millis() - timer1 > 10000) {
    Serial.printf("---------- Main: %i\n", millis() / 1000);
    Serial.printf("LinkIs Up %i, IP ", ether1.linkIsUp());
    Serial.println(ether1.getMyIP());
    printActiveStreams(STREAM_IN);
    ether1.printHosts();
    printActiveSubs();
    timer1 = millis();
  }

    // ⏱ Lecture du timer IEEE1588 toutes les 2 secondes
  if (timerPrint1588 > 2000) {
    timespec ts1;
    if (EthernetIEEE1588.readTimer(ts1)) {
      //Serial.printf("[1588] %ld s | %ld ns", ts1.tv_sec, ts1.tv_nsec);
      uint32_t sec = ts1.tv_sec;
      uint32_t nsec = ts1.tv_nsec;
      Serial.printf("[1588] %ld s | %ld ns\n", sec, nsec);
    } else {
      Serial.println("[1588] Échec lecture timer");
    }
    timerPrint1588 = 0;
  }

  checkPTP(); // Vérifie les trames PTP

  delay(100); // pour laisser QNEthernet fonctionner correctement
}
