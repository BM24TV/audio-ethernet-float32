#include <Audio.h>    // Lib audio Teensy classique (non float32)

#ifdef HAVE_AUDIO_BOARD
  #include <Wire.h>
  #include <SPI.h>
  #include <SD.h>
  #include <SerialFlash.h>
#endif

#include "control_ethernet.h"           // Contrôle matériel Ethernet (lib custom ou PJRC ?)
#include "input_net.h"                  // Entrée audio réseau (custom, style PJRC Net Audio)
#include "OpenAudio_ArduinoLibrary.h"   // Lib audio float32 moderne (non exploitée ici)
#include <QNEthernet.h>                 // Stack Ethernet avancée pour Teensy 4.x
#include "QNEthernetIEEE1588.h"         // Gestion du timer IEEE1588 PTP
using namespace qindesign::network;

// === SOCKET UDP POUR RÉCEPTION PTP ===
EthernetUDP ptpUdp;

// Timer pour affichage du timer IEEE1588 toutes les 2 secondes
elapsedMillis timerPrint1588;

// --- FONCTION DE MONITORING DES PAQUETS PTP REÇUS ---
// Ici on affiche simplement les paquets reçus sur le port PTP (319)
void checkPTP() {
  int packetSize = ptpUdp.parsePacket();
  if (packetSize > 0) {
    IPAddress ip = ptpUdp.remoteIP();
    Serial.printf("[PTP] Paquet reçu (%d octets) depuis %d.%d.%d.%d:%d\n",
                  packetSize, ip[0], ip[1], ip[2], ip[3], ptpUdp.remotePort());

    uint8_t buffer[1500];
    ptpUdp.read(buffer, packetSize);

    // Premier octet du payload = type de message PTP (ex: SYNC, DELAY_REQ...)
    uint8_t messageType = buffer[0];

    const char* ptpTypeName = "?";
    switch (messageType) {
      case 0x00: ptpTypeName = "SYNC"; break;
      case 0x01: ptpTypeName = "DELAY_REQ"; break;
      case 0x08: ptpTypeName = "FOLLOW_UP"; break;
      case 0x09: ptpTypeName = "DELAY_RESP"; break;
    }

    Serial.printf("[PTP] Type: 0x%02X (%s)\n", messageType, ptpTypeName);

    // == (OPTION INACTIVE) == Détection manuelle d'un message SYNC PTP
    /*
    if (messageType == 0x00) { // SYNC
      uint32_t ptp_now = ENET_ATVR;
      Serial.printf("[PTP] SYNC reçu — Timer PTP: %u ns\n", ptp_now);
    }
    */
    // Remarque : ce code ne fait QUE LIRE/DÉCODER les paquets PTP,
    // il n'exploite pas leur contenu pour synchroniser l'audio.
  }
}

// === PARAMÈTRES AUDIO ===
const float sample_rate_Hz = 48000.0f;
const int   audio_block_samples = 128;  // Toujours 128 pour Teensy PJRC
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

// Sortie audio I2S (float32 si supporté)
AudioOutputI2S_F32 audioOutput(audio_settings);
AudioControlEthernet ether1;         // Contrôle carte réseau (custom)
AudioInputNet in1(1);                // Entrée audio réseau

// Connexions audio internes (in1 -> audioOutput)
AudioConnection_F32 patchCord1(in1, 0, audioOutput, 0); // Gauche
AudioConnection_F32 patchCord2(in1, 0, audioOutput, 1); // Droite
AudioControlSGTL5000 sgtl;

#include "utils.h"    // Divers utilitaires (probablement pour debug)

void setup() {
  AudioMemory_F32(50);

  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  Serial.println("\n\nStarting Multi Audio Stream Test");

  // === Démarrage Ethernet (via lib custom) ===
  char myHost[] = "Teensy1";
  ether1.setHostName(myHost);
  ether1.begin();
  if (!ether1.linkIsUp())
    Serial.printf("Ethernet is disconnected");
  else
    Serial.println(ether1.getMyIP());

  // === Initialisation du timer IEEE1588 (PTP) ===
  EthernetIEEE1588.begin();
  Serial.println("IEEE1588 timer initialisé");

  // === Audio hardware ===
  in1.begin();
  sgtl.enable();
  sgtl.volume(1);
  sgtl.unmuteLineout();

  // Souscrit au flux réseau nommé "Stream1dudule"
  char s1[] = "Stream1dudule";
  in1.subscribe(s1);

  // === Écoute des trames PTP sur port 319 (standard IEEE1588) ===
  ptpUdp.begin(319); // UDP Unicast
  ptpUdp.beginMulticast(IPAddress(224, 0, 1, 129), 319);  // UDP Multicast (nécessaire pour PTP en mode multicast)

  Serial.println("Done setup");
}

#define EVERY 1000
long count = 0;
long timer1 = -3000;

void loop() {
  // --- Affichage périodique d'infos réseau ---
  if (millis() - timer1 > 10000) {
    Serial.printf("---------- Main: %i\n", millis() / 1000);
    Serial.printf("LinkIs Up %i, IP ", ether1.linkIsUp());
    Serial.println(ether1.getMyIP());
    printActiveStreams(STREAM_IN);
    ether1.printHosts();
    printActiveSubs();
    timer1 = millis();
  }

  // --- Affichage du timer IEEE1588 toutes les 2s ---
  if (timerPrint1588 > 2000) {
    timespec ts1;
    if (EthernetIEEE1588.readTimer(ts1)) {
      // Affichage du timestamp PTP courant
      uint32_t sec = ts1.tv_sec;
      uint32_t nsec = ts1.tv_nsec;
      Serial.printf("[1588] %ld s | %ld ns\n", sec, nsec);
    } else {
      Serial.println("[1588] Échec lecture timer");
    }
    timerPrint1588 = 0;
  }

  checkPTP(); // Analyse les paquets PTP reçus, les affiche uniquement

  delay(100); // Nécessaire pour laisser QNEthernet gérer ses tâches en background
}

/*
==========================
   REMARQUE IMPORTANTE 
==========================

Ce code ne synchronise PAS la lecture audio sur le timer PTP :
— Il se contente d'écouter, de décoder et d'afficher les paquets PTP du réseau.
— Il affiche le timer IEEE1588 local, mais ne cale pas l'audio sur ce timer.

=> Pour une vraie diffusion synchrone AES67, il faudrait :
   - Lire le timer PTP.
   - Bufferiser les blocs audio.
   - Lancer la lecture des blocs **au bon timestamp PTP** (comme dans le code complet avec jitter buffer !).
   - Corriger la position de lecture selon la dérive PTP (ce que ne fait pas ce code).

On peut donc :
  - Garder ce code pour monitorer/tracer le trafic PTP (démo/debug).
  - Ajouter des commentaires pour signaler qu'il NE SYNCHRONISE PAS l'audio.
*/
