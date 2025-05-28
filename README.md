# Audio over Ethernet – Projet BTS CIEL

Ce dépôt contient l’ensemble du code source développé dans le cadre du projet de diffusion audio en temps réel sur réseau Ethernet.  
Deux protocoles de diffusion ont été implémentés : **VBAN** et **AES67**.

## Structure du dépôt

<details> <summary><strong>Arborescence propre à copier</strong></summary>
/
├── vban/
│   ├── librairies/
│   ├── emetteur/
│   └── recepteur/
├── aes67/
│   ├── librairies/
│   ├── emetteur/
│   └── recepteur/
└── README.md
</details>

### Dossiers

- **`vban/`**  
  Contient tout le nécessaire pour l’implémentation du protocole VBAN sur microcontrôleur (Teensy 4.1) :
    - **`librairies/`** : dépendances nécessaires à la compilation (VBAN, UDP, audio, etc.)
    - **`emetteur/`** : code source de l’émetteur audio VBAN
    - **`recepteur/`** : code source du récepteur audio VBAN

- **`aes67/`**  
  Contient l’implémentation du protocole AES67 (RTP synchronisé PTP, compatible Audio-over-IP) :
    - **`librairies/`** : dépendances nécessaires à la compilation (RTP, PTP, UDP, etc.)
    - **`emetteur/`** : code source de l’émetteur audio AES67
    - **`recepteur/`** : code source du récepteur audio AES67

---

## À propos du projet

Le projet a pour objectif de réaliser une solution de diffusion audio multi-protocoles sur Ethernet, embarquée sur carte Teensy 4.1, incluant la synchronisation temporelle (PTP/IEEE1588) et le traitement audio en temps réel (float32).  
Chaque protocole dispose d’un émetteur et d’un récepteur indépendants.

## Utilisation

- Pour chaque protocole, se reporter au dossier correspondant.
- Les librairies nécessaires sont incluses dans chaque dossier (`librairies/`).
- Les codes sont adaptés pour être compilés et flashés sur Teensy 4.1.

## Auteurs

- Julien PRUVOST (BTS CIEL – Projet Audio sur Ethernet)

---

**Remarque** :  
Ce dépôt est fourni à titre de démonstration pour la soutenance du projet BTS.  
Les codes sont documentés et peuvent servir de base à d’autres projets de diffusion audio sur réseau.

