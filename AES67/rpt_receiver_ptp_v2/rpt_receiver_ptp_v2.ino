#include <QNEthernet.h>
#include <QNEthernetIEEE1588.h>
#include <OpenAudio_ArduinoLibrary.h>
#include <Audio.h>
#include <string>
#include <t41-ptp.h>
#include <Wire.h>
#include <TimeLib.h>

using namespace qindesign::network;

// === PARAMÈTRES RÉSEAU ===
// Configuration IP locale, masque, passerelle, DNS
IPAddress localIP(192, 168, 1, 71);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// === PARAMÈTRES MULTICAST AES67 ===
// Adresse multicast du groupe audio + port d’écoute AES67/RTP
IPAddress multicastIP(224, 0, 1, 129);
constexpr uint16_t port = 5005;

// === CONFIG UDP & RTP ===
// Socket UDP, buffer de réception (taille MTU Ethernet)
EthernetUDP udp;
constexpr size_t MAX_PACKET_SIZE = 1500;
uint8_t packetBuffer[MAX_PACKET_SIZE];
constexpr int RTP_HEADER_SIZE = 12;

// === AUDIO FLOAT32 (OpenAudio_ArduinoLibrary) ===
// Sortie I2S float32 stéréo (gauche/droite)
AudioOutputI2S_F32      i2s_out;
AudioPlayQueue_F32      queue_f32;
AudioControlSGTL5000    sgtl5000;
// Deux connexions : un canal pour chaque sortie (G/D)
AudioConnection_F32 patchCord1(queue_f32, 0, i2s_out, 0); // gauche
AudioConnection_F32 patchCord2(queue_f32, 0, i2s_out, 1); // droite

// === SYNCHRONISATION PTP & JITTER BUFFER ===
bool p2p = false, master = false, slave = true;
l3PTP ptp(master, slave, p2p);                // Instance PTP
elapsedMillis timerPrint1588;                 // Chrono logs PTP

// === BUFFER CIRCULAIRE AUDIO ===
// Permet de compenser le jitter réseau et la latence
constexpr int BUFFER_SIZE = 128;
struct AudioBlock {
  uint32_t rtpTimestamp;                      // Timestamp RTP associé au bloc
  float32_t data[AUDIO_BLOCK_SAMPLES];        // Échantillons audio
  bool valid;                                 // Bloc prêt à être lu
};
AudioBlock audioBuffer[BUFFER_SIZE];           // Buffer circulaire
volatile int bufHead = 0, bufTail = 0;        // Indices de lecture/écriture

// === PARAMÈTRES DE SYNCHRONISATION AUDIO ===
const float sampleRate = 44100.0f;            // Fréquence d'échantillonnage (doit matcher l'émetteur)

// === VARIABLES D'ANCRAGE SYNCHRO PTP/RTP ===
double first_ptp_time = 0.0;                  // Première horloge PTP reçue
uint32_t first_rtp_ts = 0;                    // Premier timestamp RTP reçu
bool anchor_set = false;                      // Si l'ancrage PTP/RTP est réalisé

// === DÉTECTION PERTE PTP ===
elapsedMillis timerSincePtpUpdate = 0;        // Temps depuis dernière MAJ PTP
long lastPtpOffset = 0;                       // Dernier offset reçu
const unsigned long PTP_TIMEOUT_MS = 5000;    // Timeout de perte PTP (5s)
bool ptp_lost = false;                        // Flag de perte de synchro PTP

// --- FONCTIONS UTILITAIRES ---
// Remet le buffer audio à zéro
void resetBuffer() {
  bufHead = bufTail = 0;
  for (int i = 0; i < BUFFER_SIZE; ++i) audioBuffer[i].valid = false;
  Serial.println("[BUFFER] Buffer réinitialisé.");
}

// Récupère l'heure PTP actuelle (protection contre glitch)
double getPtpTimeNow() {
  timespec ts;
  if (EthernetIEEE1588.readTimer(ts)) {
    double t = ts.tv_sec + ts.tv_nsec / 1e9;
    // Si horloge PTP trop vieille ou saute, reset synchro + buffer
    if (t < 1577836800.0 || fabs(t - first_ptp_time) > 10.0) {
      Serial.printf("[PTP ERROR] Anomalie horaire: %.3f s (reset anchor/buffer)\n", t);
      anchor_set = false;
      resetBuffer();
    }
    return t;
  }
  return 0.0;
}

// Calcule le timestamp RTP théorique courant (à partir du PTP et de l'ancre initiale)
uint32_t getCurrentRtpTimestamp() {
  if (!anchor_set) return 0;
  double ptp_now = getPtpTimeNow();
  double elapsed = (ptp_now - first_ptp_time) * sampleRate;
  return first_rtp_ts + (uint32_t)elapsed;
}

// Ajoute un bloc dans le buffer circulaire (avec gestion overflow)
void pushAudioBlock(uint32_t rtpTimestamp, float32_t* src) {
  int nextHead = (bufHead + 1) % BUFFER_SIZE;
  if (nextHead == bufTail) { // Overflow : écrase le plus ancien
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    Serial.println("[BUFFER] ⚠️ Overflow (jitter/latence)");
  }
  memcpy(audioBuffer[bufHead].data, src, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
  audioBuffer[bufHead].rtpTimestamp = rtpTimestamp;
  audioBuffer[bufHead].valid = true;
  bufHead = nextHead;
}

// Retire le prochain bloc prêt à jouer (si timestamp PTP a rattrapé le timestamp RTP du bloc)
bool popAudioBlock(float32_t* dest, uint32_t curRtpTimestamp) {
  if (bufTail == bufHead) return false; // Buffer vide
  AudioBlock &blk = audioBuffer[bufTail];
  // Si bloc "à temps", on le lit et on avance le buffer
  if (blk.valid && (int32_t)(curRtpTimestamp - blk.rtpTimestamp) >= 0) {
    memcpy(dest, blk.data, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
    blk.valid = false;
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    return true;
  }
  return false;
}

// --- INITIALISATION DU SYSTÈME ---
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("🔊 === Récepteur RTP L24 SYNCHRO PTP ===");

  // Initialisation réseau (Teensy + QNEthernet)
  Ethernet.setHostname("rtp-l24-receiver");
  Ethernet.begin(localIP, subnet, gateway);
  Ethernet.setDnsServerIP(dns);
  while (Ethernet.linkStatus() != LinkON) {
    Serial.println("[ETH] ⏳ En attente du lien Ethernet...");
    delay(500);
  }

  // Démarrage PTP + socket UDP
  EthernetIEEE1588.begin();
  ptp.begin();
  Serial.print("[ETH] IP locale : ");
  Serial.println(Ethernet.localIP());

  udp.beginMulticast(multicastIP, port);
  Serial.println("[RTP] ✅ En écoute sur 224.0.1.129:5005 (RTP L24)");

  // Initialisation du hardware audio (SGTL5000)
  AudioMemory(20);
  AudioMemory_F32(24);
  sgtl5000.enable();
  sgtl5000.volume(0.7);

  resetBuffer();
  Serial.println("[AUDIO] ✅ Initialisation audio terminée.");
}

// --- BOUCLE PRINCIPALE ---
void loop() {
  ptp.update();  // Mise à jour PTP (synchronisation continue)

  // --- VÉRIFICATION DÉCALAGE HORLOGE PTP ---
if (anchor_set) {
  double now = getPtpTimeNow();
  double delta = now - first_ptp_time;
  uint32_t rtpRef = first_rtp_ts + (uint32_t)(delta * sampleRate);

  // Si la différence dépasse 1 seconde ou 100000 ticks RTP (~2.2s à 44.1kHz)
  if (fabs((int32_t)(rtpRef - getCurrentRtpTimestamp())) > 100000) {
    Serial.println("[PTP ERROR] Décalage critique détecté → Reset synchro PTP/RTP");
    anchor_set = false;
    resetBuffer();
  }
}


  // --- GESTION PERTE DE SYNCHRO PTP ---
  long ptpOffset = ptp.getOffset();
  if (ptpOffset != lastPtpOffset) {
    timerSincePtpUpdate = 0;
    if (ptp_lost) {
      Serial.println("[PTP OK] PTP resynchronisé.");
      ptp_lost = false;
    }
  }
  lastPtpOffset = ptpOffset;

  if (timerSincePtpUpdate > PTP_TIMEOUT_MS && !ptp_lost) {
    ptp_lost = true;
    Serial.println("[PTP LOST] Plus de trames PTP depuis 5s, audio en SILENCE.");
  }

  // --- RÉCEPTION RTP ---
  int packetSize = udp.parsePacket();
  if (packetSize > (int)RTP_HEADER_SIZE && packetSize < (int)MAX_PACKET_SIZE) {
    int len = udp.read(packetBuffer, packetSize);
    if (len <= RTP_HEADER_SIZE) return;

    uint32_t rtpTimestamp = (packetBuffer[4] << 24) | (packetBuffer[5] << 16) |
                            (packetBuffer[6] << 8) | packetBuffer[7];

    if (!anchor_set) {
      first_ptp_time = getPtpTimeNow();
      first_rtp_ts = rtpTimestamp;
      anchor_set = true;
      Serial.printf("[SYNC] Ancrage initial : PTP=%.3f s, RTP ts=%lu\n", first_ptp_time, first_rtp_ts);
    }

    uint8_t* payload = packetBuffer + RTP_HEADER_SIZE;
    int payloadSize = len - RTP_HEADER_SIZE;
    if (payloadSize % 3 != 0) return;

    int sampleCount = payloadSize / 3;
    int samplesConsumed = 0;
    while (sampleCount - samplesConsumed >= AUDIO_BLOCK_SAMPLES) {
      float32_t blockBuf[AUDIO_BLOCK_SAMPLES];
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        uint8_t b1 = payload[(samplesConsumed + i) * 3 + 0];
        uint8_t b2 = payload[(samplesConsumed + i) * 3 + 1];
        uint8_t b3 = payload[(samplesConsumed + i) * 3 + 2];
        int32_t sample = (b1 << 16) | (b2 << 8) | b3;
        if (sample & 0x800000) sample |= 0xFF000000;
        blockBuf[i] = sample / 8388608.0f;
      }
      uint32_t blockTimestamp = rtpTimestamp + samplesConsumed;
      pushAudioBlock(blockTimestamp, blockBuf);
      samplesConsumed += AUDIO_BLOCK_SAMPLES;
    }
  }

  // --- SORTIE AUDIO SYNCHRONISÉE ---
  static elapsedMicros audioTimer = 0;
  const unsigned long audioPeriodUs = AUDIO_BLOCK_SAMPLES * 1000000UL / 44100UL;

  if (audioTimer >= audioPeriodUs) {
    audioTimer -= audioPeriodUs;

    // Calcule le timestamp cible (corrigé pour rester centré dans le buffer)
    int fill = bufHead - bufTail;
    if (fill < 0) fill += BUFFER_SIZE;
    const int targetFill = BUFFER_SIZE / 2;
    int deviation = fill - targetFill;
    int driftCorrection = constrain(deviation, -8, 8); // souple, 1 bloc = 2.9ms
    uint32_t curRtpTimestamp = getCurrentRtpTimestamp() - driftCorrection * AUDIO_BLOCK_SAMPLES;

    static float32_t playbackBuf[AUDIO_BLOCK_SAMPLES];
    float32_t* out = queue_f32.getBuffer();

    if (ptp_lost) {
      if (out) {
        memset(out, 0, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
        queue_f32.playBuffer();
        Serial.println("[AUDIO] SILENCE: PTP perdu (désynchro).");
      }
    } else if (popAudioBlock(playbackBuf, curRtpTimestamp)) {
      if (out) {
        memcpy(out, playbackBuf, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
        queue_f32.playBuffer();
        Serial.printf("[AUDIO] Bloc joué. [OFFSET] %ld ns | Buffer fill: %d\n", ptpOffset, fill);
      }
    } else if (out) {
      memset(out, 0, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
      queue_f32.playBuffer();
      Serial.printf("[AUDIO] SILENCE: Pas de bloc dispo. [OFFSET] %ld ns | Buffer fill: %d\n", ptpOffset, fill);
    }
  }

  // --- LOGS DIAGNOSTIC ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    int fill = bufHead - bufTail;
    if (fill < 0) fill += BUFFER_SIZE;
    Serial.printf("[INFO] Buffer fill: %d / %d [OFFSET] %ld ns\n", fill, BUFFER_SIZE, ptpOffset);
    lastPrint = millis();
  }
}

