#include <QNEthernet.h>
#include <QNEthernetIEEE1588.h>
#include <OpenAudio_ArduinoLibrary.h>
#include <Audio.h>
#include <string>
#include <t41-ptp.h>
#include <Wire.h>
#include <TimeLib.h>

using namespace qindesign::network;

// === Réglages réseau ===
IPAddress localIP(192, 168, 1, 71);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// === Multicast AES67 (audio) ===
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
AudioConnection_F32 patchCord1(queue_f32, 0, i2s_out, 0); // gauche
AudioConnection_F32 patchCord2(queue_f32, 0, i2s_out, 1); // droite

// === PTP & Buffer Jitter ===
bool p2p = false, master = false, slave = true;
l3PTP ptp(master, slave, p2p);
elapsedMillis timerPrint1588;

constexpr int BUFFER_SIZE = 64; // 64 blocs max
struct AudioBlock {
  uint32_t rtpTimestamp;
  float32_t data[AUDIO_BLOCK_SAMPLES];
  bool valid;
};
AudioBlock audioBuffer[BUFFER_SIZE];
volatile int bufHead = 0, bufTail = 0;

// === Paramètres de synchro ===
const float sampleRate = 44100.0f;
const uint32_t RTP_TICKS_LATENCY = (uint32_t)((30.0f * sampleRate) / 1000.0f); // 30 ms de latence cible

// === Variables d'ancrage PTP/RTP ===
double first_ptp_time = 0.0;
uint32_t first_rtp_ts = 0;
bool anchor_set = false;

// --- Fonctions utilitaires ---
void resetBuffer() {
  bufHead = bufTail = 0;
  for (int i = 0; i < BUFFER_SIZE; ++i) audioBuffer[i].valid = false;
  Serial.println("[BUFFER] Buffer réinitialisé.");
}

// Récupère l'heure PTP locale (sécures contre reset)
double getPtpTimeNow() {
  timespec ts;
  if (EthernetIEEE1588.readTimer(ts)) {
    double t = ts.tv_sec + ts.tv_nsec / 1e9;
    // Protection : reset ou glitch d'horloge PTP
    if (t < 1577836800.0 || fabs(t - first_ptp_time) > 10.0) { // 2020-01-01
      Serial.printf("[PTP ERROR] Anomalie horaire: %.3f s (reset anchor/buffer)\n", t);
      anchor_set = false;
      resetBuffer();
    }
    return t;
  }
  return 0.0;
}

// Convertit l'heure PTP locale en timestamp RTP aligné à l'ancre (premier paquet reçu)
uint32_t getCurrentRtpTimestamp() {
  if (!anchor_set) return 0;
  double ptp_now = getPtpTimeNow();
  double elapsed = (ptp_now - first_ptp_time) * sampleRate;
  return first_rtp_ts + (uint32_t)elapsed;
}

// Ajoute un bloc dans le buffer circulaire (jitter buffer)
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
  Serial.printf("[BUFFER] Block push: RTP ts=%lu\n", rtpTimestamp);
}

// Retire et lit le prochain bloc à jouer
bool popAudioBlock(float32_t* dest, uint32_t curRtpTimestamp) {
  if (bufTail == bufHead) return false; // vide
  AudioBlock &blk = audioBuffer[bufTail];
  // Log de timeline détaillé
  Serial.printf("[JITTER] curRTP=%lu | blk.ts=%lu | Δ=%ld\n", curRtpTimestamp, blk.rtpTimestamp, (int32_t)(curRtpTimestamp - blk.rtpTimestamp));
  if (blk.valid && (int32_t)(curRtpTimestamp - blk.rtpTimestamp) >= 0) {
    memcpy(dest, blk.data, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
    blk.valid = false;
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    Serial.println("[JITTER] >> Bloc POP audio !");
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("🔊 === Récepteur RTP L24 SYNCHRO PTP ===");

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

// ... [le reste du code inchangé au-dessus] ...

void loop() {
  ptp.update();

  // Debug PTP toutes les 2s
  if (timerPrint1588 > 2000) {
    timespec ts;
    if (EthernetIEEE1588.readTimer(ts)) {
      time_t sec = ts.tv_sec;
      struct tm *tm = gmtime(&sec);
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03ld",
               tm->tm_hour, tm->tm_min, tm->tm_sec, ts.tv_nsec / 1000000);
      Serial.print("[PTP] Heure PTP: ");
      Serial.println(buf);
      Serial.printf("[PTP] Offset: %ld ns | Delay: %ld ns\n", ptp.getOffset(), ptp.getDelay());
    }
    timerPrint1588 = 0;
  }

  // --- Réception RTP, bufferisation synchronisée ---
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

    if (payloadSize % 3 != 0) {
      Serial.printf("[RTP] ⚠️ Paquet audio non multiple de 3 (%d octets), ignoré\n", payloadSize);
      return;
    }
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

  // --- Lecture synchronisée du buffer audio à fréquence adaptative ---
  static unsigned long lastAudioTick = 0;
  unsigned long audioPeriodMs = 128 * 1000 / 44100; // ≈2.9 ms à 44.1kHz (valeur de base)
  // === Ajustement adaptatif ===
  int fill = bufHead - bufTail;
  if (fill < 0) fill += BUFFER_SIZE;
  int center = BUFFER_SIZE / 2;
  int deviation = fill - center;
  // On limite l'ajustement à +/- 1.0 ms pour rester "naturel" (évite le pitch-shift)
  int adjust = constrain(deviation, -8, 8); // à adapter selon la stabilité du réseau
  audioPeriodMs += adjust;

  if (anchor_set && millis() - lastAudioTick >= audioPeriodMs) {
    lastAudioTick += audioPeriodMs;
    uint32_t curRtpTimestamp = getCurrentRtpTimestamp() - RTP_TICKS_LATENCY;
    static float32_t playbackBuf[AUDIO_BLOCK_SAMPLES];
    if (popAudioBlock(playbackBuf, curRtpTimestamp)) {
      float32_t* out = queue_f32.getBuffer();
      if (out) {
        memcpy(out, playbackBuf, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
        queue_f32.playBuffer();
        Serial.println("[AUDIO] Bloc envoyé à la sortie audio.");
      }
    } else {
      // Optionnel : envoie du silence en cas de buffer vide
      float32_t* out = queue_f32.getBuffer();
      if (out) {
        memset(out, 0, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
        queue_f32.playBuffer();
        Serial.println("[AUDIO] Bloc SILENCE (buffer vide).");
      }
    }
  }

  // Diagnostic périodique du buffer
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    int fill = bufHead - bufTail;
    if (fill < 0) fill += BUFFER_SIZE;
    Serial.printf("[INFO] Buffer fill: %d blocs (latence cible: %d)\n", fill, RTP_TICKS_LATENCY / AUDIO_BLOCK_SAMPLES);
    lastPrint = millis();
  }
}
