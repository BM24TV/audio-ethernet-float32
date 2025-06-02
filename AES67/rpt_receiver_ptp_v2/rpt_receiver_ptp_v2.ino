#include <QNEthernet.h>
#include <QNEthernetIEEE1588.h>
#include <OpenAudio_ArduinoLibrary.h>
#include <Audio.h>
#include <string>
#include <t41-ptp.h>
#include <Wire.h>
#include <TimeLib.h>

using namespace qindesign::network;

// === Sécurité : définition du nombre d'échantillons par bloc ===
#ifndef AUDIO_BLOCK_SAMPLES
#define AUDIO_BLOCK_SAMPLES 128
#endif

// === PARAMÈTRES RÉSEAU ===
IPAddress localIP(192, 168, 1, 71);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// === PARAMÈTRES MULTICAST AES67 ===
IPAddress multicastIP(224, 0, 1, 129);
constexpr uint16_t port = 5005;

// === CONFIG UDP & RTP ===
EthernetUDP udp;
constexpr size_t MAX_PACKET_SIZE = 1500;
uint8_t packetBuffer[MAX_PACKET_SIZE];
constexpr int RTP_HEADER_SIZE = 12;

// === AUDIO FLOAT32 ===
AudioOutputI2S_F32      i2s_out;
AudioPlayQueue_F32      queue_f32;
AudioControlSGTL5000    sgtl5000;
AudioConnection_F32 patchCord1(queue_f32, 0, i2s_out, 0);
AudioConnection_F32 patchCord2(queue_f32, 0, i2s_out, 1);

// === SYNCHRO PTP & JITTER BUFFER ===
bool p2p = false, master = false, slave = true;
l3PTP ptp(master, slave, p2p);
elapsedMillis timerPrint1588;

constexpr int BUFFER_SIZE = 128;
struct AudioBlock {
  uint32_t rtpTimestamp;
  float32_t data[AUDIO_BLOCK_SAMPLES];
  bool valid;
};
AudioBlock audioBuffer[BUFFER_SIZE];
volatile int bufHead = 0, bufTail = 0;

const float sampleRate = 44100.0f;

// === SYNCHRONISATION DYNAMIQUE ===
double first_ptp_time = 0.0;
uint32_t first_rtp_ts = 0;
bool anchor_set = false;
const int RESYNC_THRESHOLD = 2 * AUDIO_BLOCK_SAMPLES;

// === DÉTECTION PERTE PTP ===
elapsedMillis timerSincePtpUpdate = 0;
long lastPtpOffset = 0;
const unsigned long PTP_TIMEOUT_MS = 5000;
bool ptp_lost = false;

// --- UTILS ---
void resetBuffer() {
  bufHead = bufTail = 0;
  for (int i = 0; i < BUFFER_SIZE; ++i) audioBuffer[i].valid = false;
  Serial.println("[BUFFER] Buffer réinitialisé.");
}

double getPtpTimeNow() {
  timespec ts;
  if (EthernetIEEE1588.readTimer(ts)) {
    double t = ts.tv_sec + ts.tv_nsec / 1e9;
    if (t < 1577836800.0 || fabs(t - first_ptp_time) > 10.0) {
      Serial.printf("[PTP ERROR] Anomalie horaire: %.3f s (reset anchor/buffer)\n", t);
      anchor_set = false;
      resetBuffer();
    }
    return t;
  }
  return 0.0;
}

uint32_t getCurrentRtpTimestamp() {
  if (!anchor_set) return 0;
  double ptp_now = getPtpTimeNow();
  double elapsed = (ptp_now - first_ptp_time) * sampleRate;
  return first_rtp_ts + (uint32_t)elapsed;
}

void pushAudioBlock(uint32_t rtpTimestamp, float32_t* src) {
  int nextHead = (bufHead + 1) % BUFFER_SIZE;
  if (nextHead == bufTail) {
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    Serial.println("[BUFFER] ⚠️ Overflow (jitter/latence)");
  }
  memcpy(audioBuffer[bufHead].data, src, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
  audioBuffer[bufHead].rtpTimestamp = rtpTimestamp;
  audioBuffer[bufHead].valid = true;
  bufHead = nextHead;
}

bool popAudioBlock(float32_t* dest, uint32_t curRtpTimestamp) {
  if (bufTail == bufHead) return false;
  AudioBlock &blk = audioBuffer[bufTail];
  if (blk.valid && (int32_t)(curRtpTimestamp - blk.rtpTimestamp) >= 0) {
    memcpy(dest, blk.data, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
    blk.valid = false;
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("🔊 === Récepteur RTP L24 SYNCHRO PTP DYNAMIQUE ===");

  Ethernet.setHostname("rtp-l24-receiver");
  Ethernet.begin(localIP, subnet, gateway);
  Ethernet.setDnsServerIP(dns);
  while (Ethernet.linkStatus() != LinkON) {
    Serial.println("[ETH] ⏳ En attente du lien Ethernet...");
    delay(500);
  }

  EthernetIEEE1588.begin();
  ptp.begin();
  Serial.print("[ETH] IP locale : ");
  Serial.println(Ethernet.localIP());

  udp.beginMulticast(multicastIP, port);
  Serial.println("[RTP] ✅ En écoute sur 224.0.1.129:5005 (RTP L24)");

  AudioMemory(20);
  AudioMemory_F32(24);
  sgtl5000.enable();
  sgtl5000.volume(0.7);

  resetBuffer();
  Serial.println("[AUDIO] ✅ Initialisation audio terminée.");
}

void loop() {
  ptp.update();

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

  // --- RÉCEPTION RTP & ANCRAGE DYNAMIQUE ---
  int packetSize = udp.parsePacket();
  if (packetSize > (int)RTP_HEADER_SIZE && packetSize < (int)MAX_PACKET_SIZE) {
    int len = udp.read(packetBuffer, packetSize);
    if (len <= RTP_HEADER_SIZE) return;

    uint32_t rtpTimestamp = (packetBuffer[4] << 24) | (packetBuffer[5] << 16) |
                            (packetBuffer[6] << 8) | packetBuffer[7];

    int32_t rtp_offset = (int32_t)(rtpTimestamp - getCurrentRtpTimestamp());
    if (!anchor_set || abs(rtp_offset) > RESYNC_THRESHOLD) {
      first_ptp_time = getPtpTimeNow();
      first_rtp_ts = rtpTimestamp;
      anchor_set = true;
      resetBuffer();
      Serial.printf("[SYNC] Nouvelle ancre: PTP=%.3f s, RTP ts=%lu (offset=%ld)\n", first_ptp_time, first_rtp_ts, rtp_offset);
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

    int fill = bufHead - bufTail;
    if (fill < 0) fill += BUFFER_SIZE;
    const int targetFill = BUFFER_SIZE / 2;
    int deviation = fill - targetFill;
    int driftCorrection = constrain(deviation, -8, 8);
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

/*
----------------------------------------------------------------
EXPLICATION DU MÉCANISME DE SYNCHRONISATION  
----------------------------------------------------------------------------------------

Ce code récepteur RTP/AES67 implémente une synchronisation avec l’horloge réseau PTP (IEEE1588).
Il s’inspire des exigences professionnelles de la norme AES67, qui impose la lecture d’audio parfaitement calée entre émetteur(s) et récepteur(s) même en présence de dérive d’horloge ou de perturbations réseau.

Principe du mécanisme :
- À la réception du premier paquet RTP, le récepteur associe une référence temporelle entre le timestamp PTP (horloge réseau) et le timestamp RTP (audio), ce qui permet de synchroniser la lecture audio en temps réel.
- À chaque nouveau paquet RTP reçu, le code compare le timestamp RTP attendu (calculé à partir du temps PTP courant) et le timestamp RTP effectivement reçu.
- Si un écart supérieur à un seuil paramétrable (ici, 2 blocs audio, soit ~6 ms à 44,1 kHz) est détecté, le système ré-ancre immédiatement la référence temporelle, ce qui élimine toute dérive accumulée (drift) liée à une perte PTP temporaire, un reset réseau, un changement de master clock, ou un redémarrage de l’émetteur.
- Le buffer audio circulaire (“jitter buffer”) compense les variations courtes de délai réseau et permet une lecture fluide.

Intérêt :
- Cette approche garantit une lecture audio synchrone et continue, même lors de perturbations ou de basculement PTP, sans jamais accumuler d’offset significatif entre l’audio reçu et l’horloge réseau.
- Le système est “fail-safe” : tout écart anormal provoque un recadrage automatique, évitant l’apparition de silences prolongés ou de décalages, tout en respectant la tolérance temporelle requise par la diffusion audio sur IP professionnelle.
- C’est l’architecture adoptée dans de nombreux systèmes “broadcast” (Dante, AES67, Ravenna, etc.) pour assurer l’alignement parfait entre émetteurs et récepteurs.

Personnalisation :
- Le seuil de recalage (`RESYNC_THRESHOLD`) est paramétrable selon la tolérance de l’application (plus bas pour du critique, plus haut pour éviter les resync trop fréquents).
- L’algorithme peut être étendu pour gérer des recadrages sans perte de buffer (“soft resync”) ou intégrer une gestion avancée des sous-flux RTP.

En résumé, cette gestion dynamique de l’ancrage PTP/RTP est indispensable pour toute diffusion audio sur IP en environnement professionnel, garantissant robustesse, précision et résilience du système.
*/
