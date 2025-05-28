#include <QNEthernet.h>                    // Gestion réseau Ethernet pour Teensy 4.x
#include <QNEthernetIEEE1588.h>            // Synchronisation PTP (IEEE 1588)
#include <OpenAudio_ArduinoLibrary.h>       // Traitement audio float32 performant
#include <Audio.h>
#include <string>
#include <t41-ptp.h>                       // Librairie PTP pour Teensy
#include <Wire.h>
#include <TimeLib.h>

using namespace qindesign::network;

// ======= PARAMÈTRES RESEAU =======
// Configuration IP statique du récepteur
IPAddress localIP(192, 168, 1, 71);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// ======= PARAMÈTRES MULTICAST AES67 =======
// Adresse multicast et port de réception du flux audio RTP
IPAddress multicastIP(224, 0, 1, 129);
constexpr uint16_t port = 5005;

// ======= CONFIG UDP & RTP =======
// Configuration du socket UDP et du buffer pour réception RTP
EthernetUDP udp;
constexpr size_t MAX_PACKET_SIZE = 1500;
uint8_t packetBuffer[MAX_PACKET_SIZE];
constexpr int RTP_HEADER_SIZE = 12;

// ======= CHAINE AUDIO (FLOAT32) =======
// Sortie audio float32 stéréo vers I2S
AudioOutputI2S_F32      i2s_out;
AudioPlayQueue_F32      queue_f32;
AudioControlSGTL5000    sgtl5000;

// Deux connexions : un canal float32 envoyé sur chaque voie (Gauche/Droite)
AudioConnection_F32 patchCord1(queue_f32, 0, i2s_out, 0); // Gauche
AudioConnection_F32 patchCord2(queue_f32, 0, i2s_out, 1); // Droite

// ======= SYNCHRONISATION PTP =======
bool p2p = false, master = false, slave = true;
l3PTP ptp(master, slave, p2p);
elapsedMillis timerPrint1588;

// ======= BUFFER CIRCULAIRE AUDIO (JITTER BUFFER) =======
// Permet de compenser le jitter réseau (variations du délai de transmission)
constexpr int BUFFER_SIZE = 128;
constexpr int AUDIO_BLOCK_SAMPLES = 128;
struct AudioBlock {
  uint32_t rtpTimestamp;                  // Timestamp RTP du bloc
  float32_t data[AUDIO_BLOCK_SAMPLES];    // Données audio décodées (float32)
  bool valid;                             // Indique si le bloc est prêt à jouer
};
AudioBlock audioBuffer[BUFFER_SIZE];
volatile int bufHead = 0, bufTail = 0;

// ======= PARAMÈTRES SYNCHRO & TEMPS =======
const float sampleRate = 44100.0f;        // Fréquence d’échantillonnage attendue (doit matcher l’émetteur)
double first_ptp_time = 0.0;              // Première valeur d’horloge PTP reçue
uint32_t first_rtp_ts = 0;                // Premier timestamp RTP reçu
bool anchor_set = false;                  // Flag : l'ancrage PTP/RTP est-il initialisé ?

// ======= GESTION PERTE DE SYNCHRONISATION PTP =======
elapsedMillis timerSincePtpUpdate = 0;    // Temps depuis la dernière trame PTP reçue
long lastPtpOffset = 0;                   // Dernier offset PTP reçu
const unsigned long PTP_TIMEOUT_MS = 5000;// Timeout de perte PTP (5s)
bool ptp_lost = false;                    // Flag d’alerte : perte de synchro PTP

// ======= FONCTIONS UTILITAIRES =======

// Vide le buffer audio circulaire
void resetBuffer() {
  bufHead = bufTail = 0;
  for (int i = 0; i < BUFFER_SIZE; ++i) audioBuffer[i].valid = false;
  Serial.println("[BUFFER] Buffer réinitialisé.");
}

// Récupère l'heure courante PTP (en secondes, précision sub-seconde)
double getPtpTimeNow() {
  timespec ts;
  if (EthernetIEEE1588.readTimer(ts)) {
    double t = ts.tv_sec + ts.tv_nsec / 1e9;
    // Si l’horloge est incohérente : on réinitialise tout
    if (t < 1577836800.0 || fabs(t - first_ptp_time) > 10.0) {
      Serial.printf("[PTP ERROR] Anomalie horaire: %.3f s (reset anchor/buffer)\n", t);
      anchor_set = false;
      resetBuffer();
    }
    return t;
  }
  return 0.0;
}

// Calcule le timestamp RTP théorique courant à partir de l’ancre initiale
uint32_t getCurrentRtpTimestamp() {
  if (!anchor_set) return 0;
  double ptp_now = getPtpTimeNow();
  double elapsed = (ptp_now - first_ptp_time) * sampleRate;
  return first_rtp_ts + (uint32_t)elapsed;
}

// Ajoute un bloc dans le jitter buffer (circulaire)
void pushAudioBlock(uint32_t rtpTimestamp, float32_t* src) {
  int nextHead = (bufHead + 1) % BUFFER_SIZE;
  if (nextHead == bufTail) { // Overflow : on écrase le bloc le plus ancien
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    Serial.println("[BUFFER] ⚠️ Overflow (jitter/latence)");
  }
  memcpy(audioBuffer[bufHead].data, src, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
  audioBuffer[bufHead].rtpTimestamp = rtpTimestamp;
  audioBuffer[bufHead].valid = true;
  bufHead = nextHead;
}

// Récupère le prochain bloc prêt à jouer si le timestamp PTP a rattrapé le RTP
bool popAudioBlock(float32_t* dest, uint32_t curRtpTimestamp) {
  if (bufTail == bufHead) return false; // Buffer vide
  AudioBlock &blk = audioBuffer[bufTail];
  // On lit le bloc uniquement si son timestamp RTP est “dans le passé” (ou présent)
  if (blk.valid && (int32_t)(curRtpTimestamp - blk.rtpTimestamp) >= 0) {
    memcpy(dest, blk.data, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
    blk.valid = false;
    bufTail = (bufTail + 1) % BUFFER_SIZE;
    return true;
  }
  return false;
}

// ======= INITIALISATION DU SYSTÈME =======
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("🔊 === Récepteur RTP L24 SYNCHRO PTP ===");

  // Initialisation réseau (QNEthernet)
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

  // Initialisation du codec audio (SGTL5000)
  AudioMemory(20);
  AudioMemory_F32(24);
  sgtl5000.enable();
  sgtl5000.volume(0.7);

  resetBuffer();
  Serial.println("[AUDIO] ✅ Initialisation audio terminée.");
}

// ======= BOUCLE PRINCIPALE =======
void loop() {
  ptp.update();  // Mise à jour PTP (nécessaire pour la synchro)

  // --- SUPERVISION SYNCHRO PTP/RTP ---
  if (anchor_set) {
    double now = getPtpTimeNow();
    double delta = now - first_ptp_time;
    uint32_t rtpRef = first_rtp_ts + (uint32_t)(delta * sampleRate);

    // Si trop de dérive entre PTP et RTP : reset (sécurité anti-glitch)
    if (fabs((int32_t)(rtpRef - getCurrentRtpTimestamp())) > 100000) {
      Serial.println("[PTP ERROR] Décalage critique détecté → Reset synchro PTP/RTP");
      anchor_set = false;
      resetBuffer();
    }
  }

  // --- DÉTECTION PERTE DE SYNCHRO PTP ---
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

  // --- RÉCEPTION RTP (audio multicast) ---
  int packetSize = udp.parsePacket();
  if (packetSize > RTP_HEADER_SIZE && packetSize < (int)MAX_PACKET_SIZE) {
    int len = udp.read(packetBuffer, packetSize);
    if (len <= RTP_HEADER_SIZE) return;

    // Extraction du timestamp RTP
    uint32_t rtpTimestamp = (packetBuffer[4] << 24) | (packetBuffer[5] << 16) |
                            (packetBuffer[6] << 8) | packetBuffer[7];

    // Initialisation de l'ancre PTP/RTP au premier paquet reçu
    if (!anchor_set) {
      first_ptp_time = getPtpTimeNow();
      first_rtp_ts = rtpTimestamp;
      anchor_set = true;
      Serial.printf("[SYNC] Ancrage initial : PTP=%.3f s, RTP ts=%lu\n", first_ptp_time, first_rtp_ts);
    }

    // Extraction du payload audio L24
    uint8_t* payload = packetBuffer + RTP_HEADER_SIZE;
    int payloadSize = len - RTP_HEADER_SIZE;
    if (payloadSize % 3 != 0) return; // Format L24 = 3 octets/échantillon

    int sampleCount = payloadSize / 3;
    int samplesConsumed = 0;
    while (sampleCount - samplesConsumed >= AUDIO_BLOCK_SAMPLES) {
      float32_t blockBuf[AUDIO_BLOCK_SAMPLES];
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        uint8_t b1 = payload[(samplesConsumed + i) * 3 + 0];
        uint8_t b2 = payload[(samplesConsumed + i) * 3 + 1];
        uint8_t b3 = payload[(samplesConsumed + i) * 3 + 2];
        int32_t sample = (b1 << 16) | (b2 << 8) | b3;
        if (sample & 0x800000) sample |= 0xFF000000; // Signe 24 bits
        blockBuf[i] = sample / 8388608.0f;           // Conversion en float32 [-1;1]
      }
      uint32_t blockTimestamp = rtpTimestamp + samplesConsumed;
      pushAudioBlock(blockTimestamp, blockBuf);
      samplesConsumed += AUDIO_BLOCK_SAMPLES;
    }
  }

  // --- SORTIE AUDIO SYNCHRONISÉE AVEC PTP ---
  static elapsedMicros audioTimer = 0;
  const unsigned long audioPeriodUs = AUDIO_BLOCK_SAMPLES * 1000000UL / 44100UL;

  if (audioTimer >= audioPeriodUs) {
    audioTimer -= audioPeriodUs;

    // On ajuste la position de lecture dans le buffer pour garder une latence stable
    int fill = bufHead - bufTail;
    if (fill < 0) fill += BUFFER_SIZE;
    const int targetFill = BUFFER_SIZE / 2;
    int deviation = fill - targetFill;
    int driftCorrection = constrain(deviation, -8, 8);
    uint32_t curRtpTimestamp = getCurrentRtpTimestamp() - driftCorrection * AUDIO_BLOCK_SAMPLES;

    static float32_t playbackBuf[AUDIO_BLOCK_SAMPLES];
    float32_t* out = queue_f32.getBuffer();

    if (ptp_lost) {
      // Silence si PTP perdu : on évite tout artefact sonore
      if (out) {
        memset(out, 0, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
        queue_f32.playBuffer();
        Serial.println("[AUDIO] SILENCE: PTP perdu (désynchro).");
      }
    } else if (popAudioBlock(playbackBuf, curRtpTimestamp)) {
      // Lecture normale : on joue le bloc extrait du buffer circulaire
      if (out) {
        memcpy(out, playbackBuf, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
        queue_f32.playBuffer();
        Serial.printf("[AUDIO] Bloc joué. [OFFSET] %ld ns | Buffer fill: %d\n", ptpOffset, fill);
      }
    } else if (out) {
      // Si aucun bloc n’est dispo, on joue un bloc de silence
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
