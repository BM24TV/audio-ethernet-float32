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
const int audioBlockSamples = 16;
AudioSettings_F32 audio_settings(sampleRate, audioBlockSamples);

// === BUFFER D'ÉMISSION ===
constexpr int BUFFER_SIZE = 128;
float32_t sendBuffer[BUFFER_SIZE][audioBlockSamples];
uint32_t bufferRTPts[BUFFER_SIZE];
volatile int bufHead = 0, bufTail = 0;

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
uint32_t rtpTimestamp = 0; // timestamp RTP de référence (peut être initialisé dynamiquement si besoin)

// === Ajout : Prébufferisation à la lecture ===
void bufferAudioBlock() {
  if (!audioSDPlayer.isPlaying()) return;
  if (((bufHead + 1) % BUFFER_SIZE) == bufTail) {
    // Buffer plein, on ne lit rien de plus
    return;
  }
  // Lis le bloc audio actuel (audioSDPlayer) et le place dans le buffer d'émission
  float32_t* blk = audioSDPlayer.getBuffer();
  if (blk) {
    memcpy(sendBuffer[bufHead], blk, sizeof(float32_t) * audioBlockSamples);
    bufferRTPts[bufHead] = rtpTimestamp;
    bufHead = (bufHead + 1) % BUFFER_SIZE;
    rtpTimestamp += audioBlockSamples;
    // Lance la lecture du bloc suivant si dispo
    audioSDPlayer.readNextBlock();
  }
}

void playNextTrack() {
  snprintf(filename, sizeof(filename), "M%d.WAV", currentTrack);
  Serial.print("[AUDIO] Lecture de : ");
  Serial.println(filename);

  if (!audioSDPlayer.play(filename)) {
    Serial.println("[ERREUR] Fichier non trouvé ou invalide !");
  }
  rtpTimestamp = 0; // Remise à zéro à chaque nouveau fichier

  currentTrack++;
  if (currentTrack > 8) currentTrack = 1;
  trackDelay = 0;
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

  rtpSender.begin();
  RtpSender::attachLoopToYield(&rtpSender);

  AudioMemory_F32(24);
  audioSDPlayer.begin();

  Serial.println("[AUDIO] Système prêt.");
  playNextTrack(); // Lance la première lecture
}

void loop() {
  // === Lecture de la piste suivante si terminée ===
  if (!audioSDPlayer.isPlaying() && trackDelay > 2000) {
    playNextTrack();
  }

  // === Remplissage du buffer d'émission avec les nouveaux blocs audio ===
  bufferAudioBlock();

  // === Envoi des blocs RTP au fil de l'eau, cadence basée sur l'audioBlockSamples (pour une vraie synchro PTP, timer ou top PTP ici) ===
  static elapsedMicros audioTimer = 0;
  const unsigned long audioPeriodUs = audioBlockSamples * 1000000UL / (unsigned long)sampleRate;

  if (bufTail != bufHead && audioTimer >= audioPeriodUs) {
    audioTimer -= audioPeriodUs;
    // Envoi du bloc audio depuis le buffer circulaire
    float32_t* toSend = sendBuffer[bufTail];
    uint32_t tsRtp = bufferRTPts[bufTail];
    rtpSender.send(toSend, tsRtp); // Envoi RTP L24
    bufTail = (bufTail + 1) % BUFFER_SIZE;
  }

  // === Affichage de l’heure PTP toutes les 2 secondes ===
  if (ptpPrintTimer > 2000) {
    timespec ts;
    if (EthernetIEEE1588.readTimer(ts)) {
      time_t sec = ts.tv_sec;
      struct tm *tm = gmtime(&sec);
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03ld",
               tm->tm_hour, tm->tm_min, tm->tm_sec, ts.tv_nsec / 1000000);
      Serial.print("[PTP] Heure : ");
      Serial.println(buf);
    }
    ptpPrintTimer = 0;
  }

  ptp.update();
}
