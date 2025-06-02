#include <QNEthernet.h>
#include <QNEthernetIEEE1588.h>
#include <OpenAudio_ArduinoLibrary.h>
#include "AudioToRtpSender.h"
#include "RtpSender.h"
#include <string>
#include <t41-ptp.h>

using namespace qindesign::network;

// === CONFIG RÉSEAU ===
IPAddress localIP(192, 168, 1, 70);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// === MULTICAST DESTINATION (AES67) ===
IPAddress destIP(224, 0, 1, 129);
constexpr uint16_t destPort = 5005;

// === CONFIG AUDIO ===
const float sampleRate = 44100.0f;
const int audioBlockSamples = 128;
AudioSettings_F32 audio_settings(sampleRate, audioBlockSamples);

// === COMPOSANTS AUDIO ===
AudioSDPlayer_F32  audioSDPlayer(audio_settings);
AudioOutputI2S_F32 i2s1;
RtpSender rtpSender(destIP, destPort);
AudioToRtpSender audioRtpNode(rtpSender);

AudioConnection_F32 patchOut(audioSDPlayer, 0, i2s1, 0);
AudioConnection_F32 patchRtp(audioSDPlayer, 0, audioRtpNode, 0);

// === PTP ===
bool p2p = false, master = false, slave = true;
l3PTP ptp(master, slave, p2p);
elapsedMillis ptpPrintTimer;

// === SD Card ===
#define SD_CS_PIN BUILTIN_SDCARD

// === Variables de lecture ===
int currentTrack = 1;
char filename[12];
elapsedMillis trackDelay;

void playNextTrack() {
  snprintf(filename, sizeof(filename), "M%d.WAV", currentTrack);
  Serial.print("[AUDIO] Lecture de : ");
  Serial.println(filename);

  if (!audioSDPlayer.play(filename)) {
    Serial.println("[ERREUR] Fichier non trouvé ou invalide !");
  }

  currentTrack++;
  if (currentTrack > 8) currentTrack = 1;
  trackDelay = 0;
}

void printPTPStatus() {
  // === Affichage plus représentatif du fonctionnement PTP ===
  Serial.println("=== État PTP ===");

  Serial.print("[PTP] État FSM       : ");
  Serial.println(ptp.getState()); // ex: SLAVE, MASTER, LISTENING

  Serial.print("[PTP] Offset (ns)    : ");
  Serial.println(ptp.getOffsetFromMasterNs()); // écart à la clock maître

  Serial.print("[PTP] Delay (ns)     : ");
  Serial.println(ptp.getMeanPathDelayNs()); // délai moyen aller-retour

  Serial.print("[PTP] Drift (ppb)    : ");
  Serial.println(ptp.getClockDriftPpb()); // dérive par rapport au maître

  Serial.println("====================\n");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("🔊 Émetteur AES67 (RTP L24) avec PTP en cours de démarrage...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[ERREUR] Carte SD non détectée !");
    while (1);
  }
  Serial.println("[OK] Carte SD initialisée.");

  Ethernet.setHostname("aes67-emetteur");
  Ethernet.begin(localIP, subnet, gateway);
  Ethernet.setDnsServerIP(dns);
  EthernetIEEE1588.begin();

  while (Ethernet.linkStatus() != LinkON) {
    Serial.println("⏳ En attente du lien Ethernet...");
    delay(500);
  }

  Ethernet.onLinkState([](bool state) {
    Serial.printf("[Ethernet] Link %d Mbps %s\n", Ethernet.linkSpeed(), state ? "ON" : "OFF");
    if (state) ptp.begin();
  });

  Serial.print("📡 IP locale : "); Serial.println(Ethernet.localIP());
  Serial.println("[ETH] Configuration réseau terminée.");

  // Initialisation RTP
  rtpSender.begin();
  RtpSender::attachLoopToYield(&rtpSender);

  AudioMemory_F32(24);
  audioSDPlayer.begin();

  Serial.println("[AUDIO] Système prêt.");
  playNextTrack();
}

void loop() {
  // Lancement piste suivante
  if (!audioSDPlayer.isPlaying() && trackDelay > 2000) {
    playNextTrack();
  }

  // Affichage état PTP toutes les 2s
  if (ptpPrintTimer > 2000) {
    printPTPStatus();
    ptpPrintTimer = 0;
  }

  // Synchronisation PTP
  ptp.update();
}
