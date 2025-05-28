#include <QNEthernet.h>
#include <OpenAudio_ArduinoLibrary.h>
#include "AudioToRtpSender.h"
#include "RtpSender.h"
#include <EventResponder.h>
#include <string>

// === Configuration IP fixe ===
IPAddress localIP(192, 168, 2, 210);     // Adresse IP fixe
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 1, 1);
IPAddress dns(8, 8, 8, 8);

// === Destination RTP AES67 Multicast ===
IPAddress destIP(224, 0, 1, 129); //224, 0, 1, 129
const uint16_t destPort = 5005;

// Configuration SD card
//#define SD_CS_PIN 10 // Pin CS pour la carte SD, à modifier selon votre configuration
#define SD_CS_PIN    BUILTIN_SDCARD
#define SD_MOSI_PIN  11
#define SD_SCK_PIN   13


const float sample_rate_Hz = 44100.0f;
const int   audio_block_samples = 64;  // Always 128, which is AUDIO_BLOCK_SAMPLES from AudioStream.h
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);





// === Composants audio F32 ===
AudioSynthWaveform_F32 waveform;
AudioSDPlayer_F32  audioSDPlayer(audio_settings);
//AudioOutputI2S_F32 audioOutput(audio_settings);
AudioOutputI2S_F32 i2s1;
RtpSender rtpSender(destIP, destPort);
AudioToRtpSender audioRtpNode(rtpSender);

// === Connexions F32 ===
AudioConnection_F32 patchCord(audioSDPlayer, 0, i2s1, 0);
AudioConnection_F32 patchRtp(audioSDPlayer, 0, audioRtpNode, 0);
//AudioConnection_F32 patchRtp(waveform, 0, audioRtpNode, 0);
uint32_t chrono = millis();

// Variables pour lecture des morceaux
int currentTrack = 1;
char filename[12];  // Format: "M#.WAV"
elapsedMillis trackDelay;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("AES67 RTP Sender with static IP starting...");

    // Initialisation SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Erreur : Carte SD introuvable !");
    while (1);
  }
  Serial.println("Carte SD initialisée.");

  qindesign::network::Ethernet.begin();  // DHCP
  while (!qindesign::network::Ethernet.linkStatus()) {
    Serial.println("⏳ Attente du lien Ethernet...");
    delay(500);
  }

  while (qindesign::network::Ethernet.localIP() == IPAddress(0,0,0,0)) {
    Serial.println("⏳ Attente d'une IP DHCP...");
    delay(500);
  }

  // Initialisation réseau
  //qindesign::network::Ethernet.begin(localIP, gateway, subnet, dns);


  // Initialisation RTP
  rtpSender.begin();
  RtpSender::attachLoopToYield(&rtpSender);

  // Mémoire audio
  //AudioMemory(12);
  AudioMemory_F32(24);  // 💡 nécessaire pour float32_t

  audioSDPlayer.begin();

  // Générateur audio
  waveform.begin(WAVEFORM_SINE);
  waveform.frequency(440.0f);  // La 440 Hz
  waveform.amplitude(0.9f);    // Amplitude max

}

void playNextTrack() {
  snprintf(filename, sizeof(filename), "M%d.WAV", currentTrack);
  Serial.print("Lecture de : ");
  Serial.println(filename);
  audioSDPlayer.play(filename);

  currentTrack++;
  if (currentTrack > 8) currentTrack = 1;
  trackDelay = 0;
}

void loop() {
  
    // Lecture de la piste suivante si la précédente est terminée
  if (!audioSDPlayer.isPlaying() && trackDelay > 2000) {
    playNextTrack();
  }
  
  //audioSDPlayer.play("M1.WAV");



}
