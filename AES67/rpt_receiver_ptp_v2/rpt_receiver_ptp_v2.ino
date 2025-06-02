#include <QNEthernet.h>
#include <QNEthernetIEEE1588.h>
#include <OpenAudio_ArduinoLibrary.h>
#include <Audio.h>
#include <string>
#include <t41-ptp.h>
#include <Wire.h>
#include <TimeLib.h>

using namespace qindesign::network;

// === CONFIG RÉSEAU ===
IPAddress localIP(192, 168, 1, 71);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// === MULTICAST AES67 ===
IPAddress multicastIP(224, 0, 1, 129);
constexpr uint16_t port = 5005;

// === UDP & RTP ===
EthernetUDP udp;
constexpr size_t MAX_PACKET_SIZE = 1500;
uint8_t packetBuffer[MAX_PACKET_SIZE];
constexpr int RTP_HEADER_SIZE = 12;

// === AUDIO FLOAT32 ===
AudioOutputI2S_F32 i2s_out;
AudioPlayQueue_F32 queue_f32;
AudioControlSGTL5000 sgtl5000;
AudioConnection_F32 patchCord1(queue_f32, 0, i2s_out, 0);
AudioConnection_F32 patchCord2(queue_f32, 0, i2s_out, 1);

// === PTP & BUFFER ===
bool p2p = false, master = false, slave = true;
l3PTP ptp(master, slave, p2p);
elapsedMillis timerPrint1588;

const float sampleRate = 44100.0f;
constexpr int BUFFER_SIZE = 128;

struct AudioBlock {
  uint32_t rtpTimestamp;
  float32_t data[AUDIO_BLOCK_SAMPLES];
  bool valid;
};

AudioBlock audioBuffer[BUFFER_SIZE];
volatile int bufHead = 0, bufTail = 0;

// === SYNCHRONISATION ===
double first_ptp_time = 0.0;
uint32_t first_rtp_ts = 0;
bool anchor_set = false;
const int RESYNC_THRESHOLD = 2 * AUDIO_BLOCK_SAMPLES;

// === DÉTECTION PTP PERDU ===
elapsedMillis timerSincePtpUpdate = 0;
long lastPtpOffset = 0;
const unsigned long PTP_TIMEOUT_MS = 5000;
bool ptp_lost = false;

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
    Serial.println("[BUFFER] ⚠️ Overflow");
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

  Serial.println("🔊 Récepteur RTP L24 calé sur PTP");

  Ethernet.setHostname("rtp-l24-receiver");
  Ethernet.begin(localIP, subnet, gateway);
  Ethernet.setDnsServerIP(dns);
  while (Ethernet.linkStatus() != LinkON) {
    Serial.println("[ETH] ⏳ Attente lien Ethernet...");
    delay(500);
  }

  EthernetIEEE1588.begin();
  ptp.begin();

  Serial.print("[ETH] IP : ");
  Serial.println(Ethernet.localIP());

  udp.beginMulticast(multicastIP, port);
  Serial.println("[RTP] ✅ En écoute sur 224.0.1.129:5005");

  AudioMemory(20);
  AudioMemory_F32(24);
  sgtl5000.enable();
  sgtl5000.volume(0.7);

  resetBuffer();
  Serial.println("[AUDIO] ✅ Initialisation terminée.");
}

void loop() {
  static double lastPtpPlayTime = 0.0;
  ptp.update();

  long ptpOffset = ptp.getOffset();
  if (ptpOffset != lastPtpOffset) {
    timerSincePtpUpdate = 0;
    if (ptp_lost) {
      Serial.println("[PTP OK] Resynchronisé.");
      ptp_lost = false;

      anchor_set = false;
      resetBuffer();

      lastPtpPlayTime = getPtpTimeNow();
    }
  }
  lastPtpOffset = ptpOffset;

  if (timerSincePtpUpdate > PTP_TIMEOUT_MS && !ptp_lost) {
    ptp_lost = true;
    Serial.println("[PTP LOST] PTP perdu, audio en silence.");
  }

  // --- Réception RTP
  int packetSize = udp.parsePacket();
  if (packetSize > RTP_HEADER_SIZE && packetSize < MAX_PACKET_SIZE) {
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
      Serial.printf("[SYNC] Nouvelle ancre: PTP=%.3f s, RTP=%lu (offset=%ld)\n", first_ptp_time, first_rtp_ts, rtp_offset);
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

  if (!anchor_set) return;

  // === Lecture audio calée sur PTP
  const double blockPeriod = AUDIO_BLOCK_SAMPLES / sampleRate;
  double now = getPtpTimeNow();

  if (!ptp_lost && now - lastPtpPlayTime >= blockPeriod) {
    lastPtpPlayTime += blockPeriod;

    int fill = bufHead - bufTail;
    if (fill < 0) fill += BUFFER_SIZE;
    const int targetFill = BUFFER_SIZE / 2;
    int deviation = fill - targetFill;
    int driftCorrection = constrain(deviation, -8, 8);
    uint32_t curRtpTimestamp = getCurrentRtpTimestamp() - driftCorrection * AUDIO_BLOCK_SAMPLES;

    static float32_t playbackBuf[AUDIO_BLOCK_SAMPLES];
    float32_t* out = queue_f32.getBuffer();

    if (popAudioBlock(playbackBuf, curRtpTimestamp)) {
      if (out) {
        memcpy(out, playbackBuf, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
        queue_f32.playBuffer();
        Serial.printf("[AUDIO] Bloc OK [OFFSET] %ld ns | Buffer = %d\n", ptpOffset, fill);
      }
    } else if (out) {
      memset(out, 0, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
      queue_f32.playBuffer();
      Serial.printf("[AUDIO] SILENCE: Pas de bloc [OFFSET] %ld ns | Buffer = %d\n", ptpOffset, fill);
    }
  }

  // === Log régulier
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    int fill = bufHead - bufTail;
    if (fill < 0) fill += BUFFER_SIZE;
    Serial.printf("[INFO] Buffer: %d / %d | OFFSET: %ld ns\n", fill, BUFFER_SIZE, ptpOffset);
    lastPrint = millis();
  }
}
