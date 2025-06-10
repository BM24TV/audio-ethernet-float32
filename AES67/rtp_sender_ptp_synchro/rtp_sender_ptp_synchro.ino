// === INCLUSIONS DES LIBRAIRIES ===
#include <QNEthernet.h>                     // Gestion réseau Ethernet pour Teensy 4.x
#include <QNEthernetIEEE1588.h>             // Extension pour la synchronisation PTP (IEEE 1588)
#include <OpenAudio_ArduinoLibrary.h>        // Bibliothèque audio float32 optimisée pour Teensy
#include "AudioToRtpSender.h"                // Conversion et encapsulation audio -> RTP
#include "RtpSender.h"                       // Transmission RTP en multicast
#include <string>                            // Gestion des chaînes de caractères C++
#include <t41-ptp.h>                        // Implémentation PTP sur Teensy

using namespace qindesign::network;         // Utilisation de l’espace de noms du réseau QNEthernet

// ======= CONFIGURATION RESEAU =======
IPAddress localIP(192, 168, 1, 70);         // Adresse IP locale fixe de la carte
IPAddress subnet(255, 255, 255, 0);         // Masque de sous-réseau
IPAddress gateway(192, 168, 1, 254);        // Passerelle par défaut
IPAddress dns(1, 1, 1, 1);                  // Serveur DNS (optionnel)

// ======= DIFFUSION MULTICAST AES67 =======
IPAddress destIP(224, 0, 1, 129);           // Adresse IP multicast AES67
constexpr uint16_t destPort = 5005;         // Port UDP standard AES67

// ======= CONFIGURATION AUDIO =======
const float sampleRate = 44100.0f;          // Fréquence d’échantillonnage audio (Hz)
const int audioBlockSamples = 16;           // Taille du buffer audio (en échantillons)
AudioSettings_F32 audio_settings(sampleRate, audioBlockSamples); // Paramètres audio

// ======= CHAINE AUDIO =======
AudioSDPlayer_F32  audioSDPlayer(audio_settings);   // Lecteur de fichiers WAV depuis la SD
AudioOutputI2S_F32 i2s1;                           // Sortie audio I2S (vers codec/DAC)
RtpSender rtpSender(destIP, destPort);              // Objet pour envoyer des paquets RTP
AudioToRtpSender audioRtpNode(rtpSender);           // Convertit audio float32 en paquets RTP

// Connexions audio internes
AudioConnection_F32 patchOut(audioSDPlayer, 0, i2s1, 0);      // Connecte le lecteur SD à la sortie I2S
AudioConnection_F32 patchRtp(audioSDPlayer, 0, audioRtpNode, 0); // Connecte le lecteur SD au module RTP

// ======= PROTOCOLE PTP (IEEE 1588) =======
bool p2p = false, master = false, slave = true;    // Paramètres PTP : esclave uniquement
l3PTP ptp(master, slave, p2p);                     // Création de l’objet PTP
elapsedMillis ptpPrintTimer;                       // Timer pour affichage périodique heure PTP

// ======= CARTE SD =======
#define SD_CS_PIN BUILTIN_SDCARD                  // Pin CS intégrée pour la SD sur Teensy 4.1

// ======= VARIABLES DE LECTURE =======
int currentTrack = 1;           // Piste audio en cours (M1.WAV à M8.WAV)
char filename[12];              // Nom du fichier audio à lire
elapsedMillis trackDelay;       // Anti-bouclage lors du changement de piste

// ======= FONCTION DE LECTURE DE PISTE SUIVANTE =======
void playNextTrack() {                                   // Fonction pour lire la piste suivante
  snprintf(filename, sizeof(filename), "M%d.WAV", currentTrack); // Génère le nom de fichier
  Serial.print("[AUDIO] Lecture de : ");                 // Affiche le nom du fichier
  Serial.println(filename);

  if (!audioSDPlayer.play(filename)) {                   // Tente de lire le fichier audio
    Serial.println("[ERREUR] Fichier non trouvé ou invalide !");
  }

  currentTrack++;                                        // Passe à la piste suivante
  if (currentTrack > 8) currentTrack = 1;                // Boucle sur 8 fichiers audio
  trackDelay = 0;                                        // Réinitialise le délai anti-bouclage
}

// ======= SETUP =======
void setup() {
  Serial.begin(115200);                                  // Démarre la communication série
  while (!Serial && millis() < 3000);                    // Attente que la console série soit prête

  Serial.println("🔊 Émetteur AES67 (RTP L24) avec PTP en cours de démarrage...");

  // Initialisation de la carte SD
  if (!SD.begin(SD_CS_PIN)) {                            // Tente d’initialiser la SD
    Serial.println("[ERREUR] Carte SD non détectée !");
    while (1);                                           // Stoppe le programme si SD absente
  }
  Serial.println("[OK] Carte SD initialisée.");

  // Initialisation réseau Ethernet
  Ethernet.setHostname("aes67-emetteur");                // Définit le nom de la carte sur le réseau
  Ethernet.begin(localIP, subnet, gateway);              // Configure l’IP, le masque et la passerelle
  Ethernet.setDnsServerIP(dns);                          // Configure le serveur DNS
  EthernetIEEE1588.begin();                              // Démarre la synchronisation PTP

  // Attente du lien Ethernet physique (câble branché)
  while (Ethernet.linkStatus() != LinkON) {              // Boucle tant que le lien n’est pas actif
    Serial.println("⏳ En attente du lien Ethernet...");
    delay(500);
  }

  // Gestion dynamique du lien Ethernet (hotplug)
  Ethernet.onLinkState([](bool state) {                  // Callback lors des changements du lien
    Serial.printf("[Ethernet] Link %d Mbps %s\n", Ethernet.linkSpeed(), state ? "ON" : "OFF");
    if (state) ptp.begin();                              // (Ré)initialise PTP à chaque reconnexion
  });

  Serial.print("📡 IP locale : "); Serial.println(Ethernet.localIP());
  Serial.println("[ETH] Configuration réseau terminée.");

  // Initialisation RTP
  rtpSender.begin();                                     // Démarre la transmission RTP
  RtpSender::attachLoopToYield(&rtpSender);              // Permet d’appeler loop() même pendant yield()

  // Initialisation audio
  AudioMemory_F32(24);                                   // Alloue la mémoire audio
  audioSDPlayer.begin();                                 // Prépare le lecteur SD

  Serial.println("[AUDIO] Système prêt.");
  playNextTrack();                                       // Lance la lecture de la première piste
}

// ======= LOOP PRINCIPAL =======
void loop() {
  // === Lecture automatique du fichier suivant quand la piste est terminée ===
  if (!audioSDPlayer.isPlaying() && trackDelay > 2000) { // Si la piste est terminée et délai écoulé
    playNextTrack();                                     // Passe à la piste suivante
  }

  // === Affichage périodique de l'heure PTP pour supervision ===
  if (ptpPrintTimer > 2000) {                            // Toutes les 2 secondes
    timespec ts;
    if (EthernetIEEE1588.readTimer(ts)) {                // Lit l’heure PTP actuelle
      time_t sec = ts.tv_sec;
      struct tm *tm = gmtime(&sec);
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03ld",
               tm->tm_hour, tm->tm_min, tm->tm_sec, ts.tv_nsec / 1000000);
      Serial.print("[PTP] Heure : ");
      Serial.println(buf);
    }
    ptpPrintTimer = 0;                                   // Remet le timer à zéro
  }

  // === Mise à jour PTP : traitement des trames de synchronisation réseau ===
  ptp.update();                                          // Met à jour la synchronisation PTP

  // === (OPTIONNEL) Ajout d'un timestamp PTP dans les paquets RTP ===
  // Peut être utilisé pour une supervision avancée ou synchronisation stricte
  /*
  uint64_t ts = EthernetIEEE1588.readAndClearTxTimestamp();
  rtpSender.setTimestamp(ts); // À activer si la gestion est implémentée
  */
}
