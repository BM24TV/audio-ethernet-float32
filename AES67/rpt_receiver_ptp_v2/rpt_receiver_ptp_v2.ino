#include <QNEthernet.h>                    // Inclut la bibliothèque réseau Ethernet pour Teensy 4.x
#include <QNEthernetIEEE1588.h>            // Inclut la gestion de la synchronisation PTP (IEEE 1588)
#include <OpenAudio_ArduinoLibrary.h>       // Inclut la bibliothèque de traitement audio float32
#include <Audio.h>                         // Inclut la bibliothèque audio standard Arduino
#include <string>                          // Inclut la gestion des chaînes de caractères C++
#include <t41-ptp.h>                       // Inclut la bibliothèque PTP pour Teensy
#include <Wire.h>                          // Inclut la bibliothèque I2C
#include <TimeLib.h>                       // Inclut la gestion du temps (heures, minutes...)

using namespace qindesign::network;        // Utilise l’espace de noms pour QNEthernet

// ======= PARAMÈTRES RESEAU =======
IPAddress localIP(192, 168, 1, 72);        // Adresse IP statique du récepteur
IPAddress subnet(255, 255, 255, 0);        // Masque de sous-réseau
IPAddress gateway(192, 168, 1, 254);       // Passerelle réseau
IPAddress dns(1, 1, 1, 1);                 // Serveur DNS

// ======= PARAMÈTRES MULTICAST AES67 =======
IPAddress multicastIP(224, 0, 1, 129);     // Adresse multicast pour la réception audio AES67
constexpr uint16_t port = 5005;            // Port UDP d’écoute du flux audio RTP

// ======= CONFIG UDP & RTP =======
EthernetUDP udp;                           // Instance de socket UDP
constexpr size_t MAX_PACKET_SIZE = 1500;   // Taille maximale d’un paquet réseau
uint8_t packetBuffer[MAX_PACKET_SIZE];     // Buffer de réception des paquets UDP
constexpr int RTP_HEADER_SIZE = 12;        // Taille de l’en-tête RTP

// ======= CHAINE AUDIO (FLOAT32) =======
AudioOutputI2S_F32      i2s_out;           // Sortie audio I2S en float32
AudioPlayQueue_F32      queue_f32;         // File d’attente pour les blocs audio float32
AudioControlSGTL5000    sgtl5000;          // Contrôle du codec audio

AudioConnection_F32 patchCord1(queue_f32, 0, i2s_out, 0); // Connecte le canal 0 à la sortie gauche
AudioConnection_F32 patchCord2(queue_f32, 0, i2s_out, 1); // Connecte le canal 0 à la sortie droite

// ======= SYNCHRONISATION PTP =======
bool p2p = false, master = false, slave = true; // Paramètres du mode PTP
l3PTP ptp(master, slave, p2p);                  // Instance de la gestion PTP
elapsedMillis timerPrint1588;                   // Timer pour affichage périodique

// ======= BUFFER CIRCULAIRE AUDIO (JITTER BUFFER) =======
constexpr int BUFFER_SIZE = 128;                // Taille du buffer circulaire audio
struct AudioBlock {                             // Structure d’un bloc audio
  uint32_t rtpTimestamp;                        // Timestamp RTP du bloc
  float32_t data[AUDIO_BLOCK_SAMPLES];          // Données audio décodées (float32)
  bool valid;                                   // Indique si le bloc est prêt à jouer
};
AudioBlock audioBuffer[BUFFER_SIZE];            // Buffer circulaire de blocs audio
volatile int bufHead = 0, bufTail = 0;          // Pointeurs de tête et queue du buffer

// ======= PARAMÈTRES SYNCHRO & TEMPS =======
const float sampleRate = 44100.0f;              // Fréquence d’échantillonnage attendue
double first_ptp_time = 0.0;                    // Première valeur reçue de l’horloge PTP
uint32_t first_rtp_ts = 0;                      // Premier timestamp RTP reçu
bool anchor_set = false;                        // Indique si la synchronisation PTP/RTP est initialisée

// ======= GESTION PERTE DE SYNCHRONISATION PTP =======
elapsedMillis timerSincePtpUpdate = 0;          // Temps écoulé depuis la dernière trame PTP reçue
long lastPtpOffset = 0;                         // Dernier offset PTP reçu
const unsigned long PTP_TIMEOUT_MS = 5000;      // Timeout en ms pour perte PTP
bool ptp_lost = false;                          // Indicateur de perte de synchro PTP

// ======= FONCTIONS UTILITAIRES =======

// Vide le buffer audio circulaire
void resetBuffer() {
  bufHead = bufTail = 0;                        // Réinitialise les pointeurs du buffer
  for (int i = 0; i < BUFFER_SIZE; ++i) audioBuffer[i].valid = false; // Invalide tous les blocs
  Serial.println("[BUFFER] Buffer réinitialisé."); // Message de log
}

// Récupère l'heure courante PTP (en secondes)
double getPtpTimeNow() {
  timespec ts;                                  // Structure pour le temps PTP
  if (EthernetIEEE1588.readTimer(ts)) {         // Lecture du timer PTP
    double t = ts.tv_sec + ts.tv_nsec / 1e9;    // Conversion en secondes
    if (t < 1577836800.0 || fabs(t - first_ptp_time) > 10.0) { // Détection anomalie
      Serial.printf("[PTP ERROR] Anomalie horaire: %.3f s (reset anchor/buffer)\n", t);
      anchor_set = false;                       // Réinitialise l’ancre
      resetBuffer();                            // Vide le buffer audio
    }
    return t;                                   // Retourne le temps courant PTP
  }
  return 0.0;                                   // Retourne 0 si échec de lecture
}

// Calcule le timestamp RTP courant à partir de la référence initiale
uint32_t getCurrentRtpTimestamp() {
  if (!anchor_set) return 0;                    // Si pas d’ancre, retourne 0
  double ptp_now = getPtpTimeNow();             // Récupère l’heure PTP actuelle
  double elapsed = (ptp_now - first_ptp_time) * sampleRate; // Durée écoulée en échantillons
  return first_rtp_ts + (uint32_t)elapsed;      // Timestamp RTP actuel
}

// Ajoute un bloc dans le jitter buffer circulaire
void pushAudioBlock(uint32_t rtpTimestamp, float32_t* src) {
  int nextHead = (bufHead + 1) % BUFFER_SIZE;   // Calcul du prochain index de tête
  if (nextHead == bufTail) {                    // Si buffer plein (overflow)
    bufTail = (bufTail + 1) % BUFFER_SIZE;      // Avance la queue (perte du bloc le plus ancien)
    Serial.println("[BUFFER] ⚠️ Overflow (jitter/latence)");
  }
  memcpy(audioBuffer[bufHead].data, src, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES); // Copie les données audio
  audioBuffer[bufHead].rtpTimestamp = rtpTimestamp; // Stocke le timestamp RTP
  audioBuffer[bufHead].valid = true;            // Marque le bloc comme valide
  bufHead = nextHead;                           // Avance la tête du buffer
}

// Récupère le prochain bloc prêt à jouer si le timestamp est atteint
bool popAudioBlock(float32_t* dest, uint32_t curRtpTimestamp) {
  if (bufTail == bufHead) return false;         // Si buffer vide, retourne false
  AudioBlock &blk = audioBuffer[bufTail];       // Récupère le bloc courant
  if (blk.valid && (int32_t)(curRtpTimestamp - blk.rtpTimestamp) >= 0) { // Bloc prêt à jouer
    memcpy(dest, blk.data, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES); // Copie les données audio
    blk.valid = false;                          // Invalide le bloc
    bufTail = (bufTail + 1) % BUFFER_SIZE;      // Avance la queue
    return true;                                // Bloc récupéré avec succès
  }
  return false;                                 // Aucun bloc disponible
}

// ======= INITIALISATION DU SYSTÈME =======
void setup() {
  Serial.begin(115200);                         // Démarre la communication série à 115200 bauds
  while (!Serial && millis() < 3000);           // Attend l’ouverture du port série

  Serial.println("🔊 === Récepteur RTP L24 SYNCHRO PTP ==="); // Message de bienvenue

  Ethernet.setHostname("rtp-l24-receiver");     // Définit le nom réseau
  Ethernet.begin(localIP, subnet, gateway);     // Démarre Ethernet avec IP statique
  Ethernet.setDnsServerIP(dns);                 // Définit le serveur DNS
  while (Ethernet.linkStatus() != LinkON) {     // Attend la connexion Ethernet
    Serial.println("[ETH] ⏳ En attente du lien Ethernet...");
    delay(500);
  }

  EthernetIEEE1588.begin();                     // Démarre la synchronisation PTP
  ptp.begin();                                 // Initialise la gestion PTP
  Serial.print("[ETH] IP locale : ");           // Affiche l’IP locale
  Serial.println(Ethernet.localIP());

  udp.beginMulticast(multicastIP, port);        // Démarre la réception multicast UDP
  Serial.println("[RTP] ✅ En écoute sur 224.0.1.129:5005 (RTP L24)");

  AudioMemory(20);                             // Alloue la mémoire audio standard
  AudioMemory_F32(24);                         // Alloue la mémoire audio float32
  sgtl5000.enable();                           // Active le codec SGTL5000
  sgtl5000.volume(0.7);                        // Définit le volume

  resetBuffer();                               // Vide le buffer audio
  Serial.println("[AUDIO] ✅ Initialisation audio terminée.");
}

// ======= BOUCLE PRINCIPALE =======
void loop() {
  ptp.update();                                // Met à jour la synchro PTP

  // --- SUPERVISION SYNCHRO PTP/RTP ---
  if (anchor_set) {
    double now = getPtpTimeNow();              // Récupère l’heure PTP
    double delta = now - first_ptp_time;       // Calcule le temps écoulé depuis l’ancrage
    uint32_t rtpRef = first_rtp_ts + (uint32_t)(delta * sampleRate); // Calcule le timestamp RTP attendu

    if (fabs((int32_t)(rtpRef - getCurrentRtpTimestamp())) > 100000) { // Vérifie la dérive
      Serial.println("[PTP ERROR] Décalage critique détecté → Reset synchro PTP/RTP");
      anchor_set = false;                      // Réinitialise l’ancrage
      resetBuffer();                           // Vide le buffer
    }
  }

  // --- DÉTECTION PERTE DE SYNCHRO PTP ---
  long ptpOffset = ptp.getOffset();            // Récupère l’offset PTP courant
  if (ptpOffset != lastPtpOffset) {            // Si l’offset a changé
    timerSincePtpUpdate = 0;                   // Réinitialise le timer
    if (ptp_lost) {                            // Si la synchro était perdue
      Serial.println("[PTP OK] PTP resynchronisé.");
      ptp_lost = false;                        // Marque la synchro comme retrouvée
    }
  }
  lastPtpOffset = ptpOffset;                   // Mémorise le dernier offset

  if (timerSincePtpUpdate > PTP_TIMEOUT_MS && !ptp_lost) { // Si trop de temps sans PTP
    ptp_lost = true;                           // Marque la perte de synchro
    Serial.println("[PTP LOST] Plus de trames PTP depuis 5s, audio en SILENCE.");
  }

  // --- RÉCEPTION RTP (audio multicast) ---
  int packetSize = udp.parsePacket();          // Vérifie si un paquet UDP a été reçu
  if (packetSize > RTP_HEADER_SIZE && packetSize < (int)MAX_PACKET_SIZE) { // Vérifie la taille
    int len = udp.read(packetBuffer, packetSize); // Lit le paquet dans le buffer
    if (len <= RTP_HEADER_SIZE) return;        // Ignore si trop petit

    uint32_t rtpTimestamp = (packetBuffer[4] << 24) | (packetBuffer[5] << 16) | // Extrait le timestamp RTP
                            (packetBuffer[6] << 8) | packetBuffer[7];

    if (!anchor_set) {                         // Si la synchro n’est pas initialisée
      first_ptp_time = getPtpTimeNow();        // Mémorise l’heure PTP actuelle
      first_rtp_ts = rtpTimestamp;             // Mémorise le timestamp RTP initial
      anchor_set = true;                       // Active l’ancrage
      Serial.printf("[SYNC] Ancrage initial : PTP=%.3f s, RTP ts=%lu\n", first_ptp_time, first_rtp_ts);
    }

    uint8_t* payload = packetBuffer + RTP_HEADER_SIZE; // Pointe sur la charge utile audio
    int payloadSize = len - RTP_HEADER_SIZE;           // Taille du payload
    if (payloadSize % 3 != 0) return;                  // Vérifie le format L24 (3 octets/échantillon)

    int sampleCount = payloadSize / 3;                 // Nombre d’échantillons dans le paquet
    int samplesConsumed = 0;                           // Nombre d’échantillons consommés
    while (sampleCount - samplesConsumed >= AUDIO_BLOCK_SAMPLES) { // Tant qu’il reste un bloc complet
      float32_t blockBuf[AUDIO_BLOCK_SAMPLES];         // Buffer temporaire pour un bloc audio
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {  // Pour chaque échantillon du bloc
        uint8_t b1 = payload[(samplesConsumed + i) * 3 + 0]; // Premier octet
        uint8_t b2 = payload[(samplesConsumed + i) * 3 + 1]; // Deuxième octet
        uint8_t b3 = payload[(samplesConsumed + i) * 3 + 2]; // Troisième octet
        int32_t sample = (b1 << 16) | (b2 << 8) | b3;        // Assemble 24 bits
        if (sample & 0x800000) sample |= 0xFF000000;         // Signe pour 24 bits
        blockBuf[i] = sample / 8388608.0f;                   // Conversion en float32 [-1,1]
      }
      uint32_t blockTimestamp = rtpTimestamp + samplesConsumed; // Timestamp du bloc
      pushAudioBlock(blockTimestamp, blockBuf);              // Ajoute le bloc au buffer circulaire
      samplesConsumed += AUDIO_BLOCK_SAMPLES;                // Avance l’index des échantillons
    }
  }

  // --- SORTIE AUDIO SYNCHRONISÉE AVEC PTP ---
  static elapsedMicros audioTimer = 0;                      // Timer pour synchroniser la sortie audio
  const unsigned long audioPeriodUs = AUDIO_BLOCK_SAMPLES * 1000000UL / 44100UL; // Durée d’un bloc audio

  if (audioTimer >= audioPeriodUs) {                        // Si il est temps de jouer un bloc
    audioTimer -= audioPeriodUs;                            // Met à jour le timer

    int fill = bufHead - bufTail;                           // Calcule le remplissage du buffer
    if (fill < 0) fill += BUFFER_SIZE;
    const int targetFill = BUFFER_SIZE / 2;                 // Remplissage cible (latence nominale)
    int deviation = fill - targetFill;                      // Déviation de la latence
    int driftCorrection = constrain(deviation, -8, 8);      // Correction de dérive limitée
    uint32_t curRtpTimestamp = getCurrentRtpTimestamp() - driftCorrection * AUDIO_BLOCK_SAMPLES; // Calcule le timestamp à lire

    static float32_t playbackBuf[AUDIO_BLOCK_SAMPLES];      // Buffer temporaire pour lecture
    float32_t* out = queue_f32.getBuffer();                 // Réserve un buffer de sortie

    if (ptp_lost) {                                         // Si perte de synchro PTP
      if (out) {
        memset(out, 0, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES); // Remplit de silence
        queue_f32.playBuffer();                             // Joue le silence
        Serial.println("[AUDIO] SILENCE: PTP perdu (désynchro).");
      }
    } else if (popAudioBlock(playbackBuf, curRtpTimestamp)) { // Si un bloc est dispo
      if (out) {
        memcpy(out, playbackBuf, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES); // Copie le bloc audio
        queue_f32.playBuffer();                             // Joue le bloc
        Serial.printf("[AUDIO] Bloc joué. [OFFSET] %ld ns | Buffer fill: %d\n", ptpOffset, fill);
      }
    } else if (out) {                                       // Sinon, joue un bloc de silence
      memset(out, 0, sizeof(float32_t) * AUDIO_BLOCK_SAMPLES);
      queue_f32.playBuffer();
      Serial.printf("[AUDIO] SILENCE: Pas de bloc dispo. [OFFSET] %ld ns | Buffer fill: %d\n", ptpOffset, fill);
    }
  }

  // --- LOGS DIAGNOSTIC ---
  static unsigned long lastPrint = 0;                       // Timer pour logs périodiques
  if (millis() - lastPrint > 1000) {                        // Toutes les secondes
    int fill = bufHead - bufTail;                           // Calcule le remplissage du buffer
    if (fill < 0) fill += BUFFER_SIZE;
    Serial.printf("[INFO] Buffer fill: %d / %d [OFFSET] %ld ns\n", fill, BUFFER_SIZE, ptpOffset); // Affiche l’état du buffer
    lastPrint = millis();                                   // Met à jour le timer des logs
  }
}
