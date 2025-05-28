#include <QNEthernet.h>                     // Gestion réseau Ethernet pour Teensy 4.x
#include <QNEthernetIEEE1588.h>             // Extension pour la synchronisation PTP (IEEE 1588)
#include <OpenAudio_ArduinoLibrary.h>        // Bibliothèque audio float32 optimisée pour Teensy
#include "AudioToRtpSender.h"                // Conversion et encapsulation audio -> RTP
#include "RtpSender.h"                       // Transmission RTP en multicast
#include <string>
#include <t41-ptp.h>                        // Implémentation PTP sur Teensy

using namespace qindesign::network;

// ======= CONFIGURATION RESEAU =======
// Paramètres IP fixes de la carte (adaptez selon votre réseau)
IPAddress localIP(192, 168, 1, 70);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 254);
IPAddress dns(1, 1, 1, 1);

// ======= DIFFUSION MULTICAST AES67 =======
// Adresse IP multicast AES67 (224.0.1.129) et port standard (5005)
IPAddress destIP(224, 0, 1, 129);
constexpr uint16_t destPort = 5005;

// ======= CONFIGURATION AUDIO =======
// Paramètres audio : fréquence d'échantillonnage (Hz) et taille du buffer (échantillons)
const float sampleRate = 44100.0f;
const int audioBlockSamples = 128;
AudioSettings_F32 audio_settings(sampleRate, audioBlockSamples);

// ======= CHAINE AUDIO =======
// Lecteur de fichier WAV sur carte SD
AudioSDPlayer_F32  audioSDPlayer(audio_settings);
// Sortie audio vers convertisseur I2S (vers DAC/codec)
AudioOutputI2S_F32 i2s1;
// Module d'encapsulation RTP (transmission réseau)
RtpSender rtpSender(destIP, destPort);
// Node qui convertit l'audio float32 en paquets RTP
AudioToRtpSender audioRtpNode(rtpSender);

// Connexions audio internes
AudioConnection_F32 patchOut(audioSDPlayer, 0, i2s1, 0);      // SD -> I2S
AudioConnection_F32 patchRtp(audioSDPlayer, 0, audioRtpNode, 0); // SD -> RTP

// ======= PROTOCOLE PTP (IEEE 1588) =======
// Initialisation du module PTP (uniquement esclave)
bool p2p = false, master = false, slave = true;
l3PTP ptp(master, slave, p2p);
elapsedMillis ptpPrintTimer;   // Timer pour l'affichage régulier de l'heure PTP

// ======= CARTE SD =======
#define SD_CS_PIN BUILTIN_SDCARD  // Pin CS intégrée pour la SD sur Teensy 4.1

// ======= VARIABLES DE LECTURE =======
int currentTrack = 1;           // Piste audio en cours (M1.WAV à M8.WAV)
char filename[12];              // Nom du fichier audio à lire
elapsedMillis trackDelay;       // Anti-bouclage lors du changement de piste

// ======= FONCTION DE LECTURE DE PISTE SUIVANTE =======
// Lance la lecture du prochain fichier audio sur la SD (nommé M1.WAV à M8.WAV)
void playNextTrack() {
  snprintf(filename, sizeof(filename), "M%d.WAV", currentTrack);
  Serial.print("[AUDIO] Lecture de : ");
  Serial.println(filename);

  if (!audioSDPlayer.play(filename)) {
    Serial.println("[ERREUR] Fichier non trouvé ou invalide !");
  }

  currentTrack++;
  if (currentTrack > 8) currentTrack = 1;  // Boucle sur les 8 fichiers audio
  trackDelay = 0;
}

// ======= SETUP =======
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Attente console ouverte (optionnel)

  Serial.println("🔊 Émetteur AES67 (RTP L24) avec PTP en cours de démarrage...");

  // Initialisation de la carte SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[ERREUR] Carte SD non détectée !");
    while (1);  // Arrêt si SD non disponible
  }
  Serial.println("[OK] Carte SD initialisée.");

  // Initialisation réseau Ethernet
  Ethernet.setHostname("aes67-emetteur");
  Ethernet.begin(localIP, subnet, gateway);
  Ethernet.setDnsServerIP(dns);
  EthernetIEEE1588.begin();

  // Attente du lien Ethernet physique (câble branché)
  while (Ethernet.linkStatus() != LinkON) {
    Serial.println("⏳ En attente du lien Ethernet...");
    delay(500);
  }

  // Gestion dynamique du lien Ethernet (hotplug)
  Ethernet.onLinkState([](bool state) {
    Serial.printf("[Ethernet] Link %d Mbps %s\n", Ethernet.linkSpeed(), state ? "ON" : "OFF");
    if (state) ptp.begin(); // (Ré)initialise PTP à chaque reconnexion réseau
  });

  Serial.print("📡 IP locale : "); Serial.println(Ethernet.localIP());
  Serial.println("[ETH] Configuration réseau terminée.");

  // Initialisation RTP
  rtpSender.begin();
  RtpSender::attachLoopToYield(&rtpSender); // Permet d’appeler loop() même lors de yield()

  // Initialisation audio
  AudioMemory_F32(24);        // Allocation du buffer audio
  audioSDPlayer.begin();      // Prépare le lecteur SD

  Serial.println("[AUDIO] Système prêt.");
  playNextTrack();            // Lance la première lecture
}

// ======= LOOP PRINCIPAL =======
void loop() {
  // === Lecture automatique du fichier suivant quand la piste est terminée ===
  if (!audioSDPlayer.isPlaying() && trackDelay > 2000) {
    playNextTrack();
  }

  // === Affichage périodique de l'heure PTP pour supervision ===
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

  // === Mise à jour PTP : traitement des trames de synchronisation réseau ===
  ptp.update();

  // === (OPTIONNEL) Ajout d'un timestamp PTP dans les paquets RTP ===
  // Peut être utilisé pour une supervision avancée ou synchronisation stricte
  /*
  uint64_t ts = EthernetIEEE1588.readAndClearTxTimestamp();
  rtpSender.setTimestamp(ts); // À activer si la gestion est implémentée
  */
}
