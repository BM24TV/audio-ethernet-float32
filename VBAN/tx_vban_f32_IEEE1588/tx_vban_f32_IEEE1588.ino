// Test multi-flux audio pour Teensy Ethernet Audio Library
// Nécessite : 2 Teensy 4.1 avec adaptateur Ethernet, ou 1 Teensy 4.1 + un PC (ex: Voicemeeter)

// ---------------
// CONFIGURATIONS
// ---------------

#define HAVE_AUDIO_BOARD // Définir si un Teensy Audio Board est connecté (optionnel mais inoffensif)
#include <Audio.h>

#ifdef HAVE_AUDIO_BOARD
  #include <Wire.h>
  #include <SPI.h>
  #include <SD.h>
  #include <SerialFlash.h>
#endif

#include "control_ethernet.h"       // Contrôle matériel Ethernet (lib spécifique)
#include "input_net.h"              // Gestion des flux entrants réseau
#include "output_net.h"             // Gestion des flux sortants réseau
#include "OpenAudio_ArduinoLibrary.h" // Audio float32, mais utilisé ici via AudioSDPlayer_F32
#include <string>
#include "QNEthernetIEEE1588.h"     // Gestion du timer PTP/IEEE1588
using namespace qindesign::network;

// ---------------
// CARTE SD
// ---------------

#define SD_CS_PIN    BUILTIN_SDCARD
#define SD_MOSI_PIN  11
#define SD_SCK_PIN   13

// ---------------
// AUDIO (float32, 48 kHz, 128 samples)
// ---------------

const float sample_rate_Hz = 48000.0f;
const int   audio_block_samples = 128;
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

// === Composants audio ===
AudioSDPlayer_F32  audioSDPlayer(audio_settings); // Lecture fichier WAV depuis carte SD
AudioOutputI2S_F32 audioOutput(audio_settings);   // Sortie I2S stéréo

AudioControlEthernet   ether1;       // Contrôle matériel réseau
AudioOutputNet         out1(1);      // Sortie réseau (VBAN, custom)
AudioSynthSineCosine_F32    sineCos; // Générateur de test (onde sinusoïdale)

// === Connexions audio internes ===
AudioConnection_F32      patchCord1(audioSDPlayer, 0, audioOutput, 0); // SD -> I2S gauche
AudioConnection_F32      patchCord2(audioSDPlayer, 1, audioOutput, 1); // SD -> I2S droite
AudioConnection_F32      patchCord3(audioSDPlayer, 0, out1, 0);        // SD -> flux réseau


#include "utils.h" // Fonctions de debug/affichage (streams, hosts, subs...)

// ---------------
// Variables lecture SD
// ---------------

int currentTrack = 1;
char filename[12];  // Nom du fichier audio : "M1.WAV", "M2.WAV", etc.
elapsedMillis trackDelay; // Timer anti-bouclage piste

// ---------------
// Timer PTP (IEEE1588)
// ---------------

elapsedMillis timerPrint1588;


// ========================================
// SETUP - Initialisation matérielle et audio
// ========================================
void setup() 
{
  Serial.begin(115200);
  while (!Serial && millis() < 5000) 
  {
    delay(100);
  }
  Serial.println("\n\n[START] Multi Audio Stream + IEEE1588");

  // --- Initialisation de la carte SD ---
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Erreur : Carte SD introuvable !");
    while (1); // Arrêt en cas d’erreur
  }
  Serial.println("Carte SD initialisée.");

  // --- Initialisation de la mémoire audio ---
  AudioMemory_F32(30, audio_settings);

  // Générateur d’onde sinusoïdale (optionnel, pour tests)
  float fr = 600.0f;
  sineCos.amplitude(0.2); 
  sineCos.frequency(fr * 44117.647f / sample_rate_Hz);

  // --- Initialisation Ethernet ---
  char myHost[] = "Teensy1";
  ether1.setHostName(myHost);
  ether1.begin();
  if(!ether1.linkIsUp())
    Serial.printf("Ethernet is disconnected");
  else
    Serial.println(ether1.getMyIP());

  // --- Sortie réseau : on "subscribe" au flux sortant nommé "Stream1dudule" ---
  std::string streamName = "Stream1dudule";
  out1.subscribe(streamName.data());  
  out1.begin();

  // --- Préparation du lecteur SD audio ---
  audioSDPlayer.begin();

  // --- Initialisation IEEE1588 (PTP) ---
  EthernetIEEE1588.begin();
  Serial.println("IEEE1588 timer initialisé");

  trackDelay = 0;
  timerPrint1588 = 0;

  Serial.println("Done setup");
}


// ========================================
// FONCTION pour lire la prochaine piste
// ========================================
void playNextTrack() {
  snprintf(filename, sizeof(filename), "M%d.WAV", currentTrack);
  Serial.print("Lecture de : ");
  Serial.println(filename);
  audioSDPlayer.play(filename);

  currentTrack++;
  if (currentTrack > 8) currentTrack = 1; // Boucle sur 8 fichiers
  trackDelay = 0;
}


// ========================================
// BOUCLE PRINCIPALE
// ========================================
static uint32_t timer1 = 0;

void loop() 
{
  // --- Gestion des pistes audio SD ---
  // Lance la piste suivante quand la précédente est terminée
  if (!audioSDPlayer.isPlaying() && trackDelay > 2000) {
    playNextTrack();
  }

  // --- Affiche périodiquement l’état du réseau et des flux ---
  static elapsedMillis timerEthernet;
  if(millis() - timer1 > 10000)
  {
    Serial.printf("LinkIs Up %i, IP ", ether1.linkIsUp());
    Serial.println(ether1.getMyIP());                         
    printActiveStreams(STREAM_OUT);
    ether1.printHosts();
    printActiveSubs();
    timer1 = millis();
  }

  // --- Affiche l'heure du timer IEEE1588 (PTP) toutes les 2s ---
  if (timerPrint1588 > 2000) {
    timespec ts;
    if (EthernetIEEE1588.readTimer(ts)) {
      Serial.printf("[1588] %ld s | %ld ns\n", ts.tv_sec, ts.tv_nsec);
    } else {
      Serial.println("[1588] Échec lecture timer");
    }
    timerPrint1588 = 0;
  }

  // --- Affiche aussi l'heure PTP chaque seconde ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {  
    lastPrint = millis();
    timespec ts;
    if (EthernetIEEE1588.readTimer(ts)) {
      Serial.print("Temps actuel - Secondes: ");
      Serial.print(ts.tv_sec);
      Serial.print(" | Nanosecondes: ");
      Serial.println(ts.tv_nsec);
    } else {
      Serial.println("❌ Échec de lecture du timer IEEE1588");
    }
  }

  // Les délais réguliers ne perturbent pas QNEthernet
  delay(100); // laisse QNEthernet/yield() faire ses tâches réseau

  delay(100);
}

/*
=======================================================
   NOTE IMPORTANTE 
=======================================================

Ce code ÉMET un flux audio sur le réseau (VBAN/Net), mais il ne synchronise PAS la sortie audio 
sur l’horloge réseau PTP/IEEE1588 :

— Il initialise le timer PTP, et affiche l’heure PTP courante pour le debug.
— AUCUNE synchronisation stricte de la lecture ni de l’envoi des paquets n’est réalisée sur le timer PTP.
— Pour une diffusion AES67 synchrone et professionnelle, il faudrait bufferiser l’audio,
   puis envoyer chaque paquet audio au moment précis du timestamp PTP visé (cf. code AES67 RTP/SDP).

Ce code est donc un **transmetteur simple**, parfait pour des tests multi-flux ou pour faire du monitoring réseau/audio,
mais il N’EXPLOITE PAS LE PTP pour caler l’audio sur plusieurs appareils, contrairement à un système “broadcast pro” synchrone.
*/
