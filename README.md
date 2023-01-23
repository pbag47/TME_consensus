# TME_consensus
___

TME_consensus est un projet Python destiné à l'enseignement du module d'automatique à des étudiants en école d'ingénieur.
Il permet de mettre en oeuvre plusieurs drones Bitcraze Crazyflie dans un essaim, et sert de support au développement de lois de commandes
de consensus automatisés entre les drones.

Ce fichier détaille la configuration et l'installation des différents modules en partant de "zéro" (PC vierge sans système d'exploitation et connexion Internet fiable).

## Pré-requis
Ce projet a été conçu pour fonctionner en Python 3.10 avec le logiciel Pycharm Community Edition sur une machine Ubuntu 22.04 "Jammy Jellyfish" LTS.


### Installation d'Ubuntu
- Créer une clé USB bootable contenant Ubuntu 22.04 (voir documentation sur le site officiel Linux) ;
- Après avoir branché cette clé bootable sur le PC dédié au projet, démarrer le PC et ouvrir le menu de démarrage ou le menu du BIOS ;
- Configurer le périphérique démarrage pour que l'ordinateur démarre avec l'OS contenu dans la clé USB, puis enregistrer et rebooter le PC ;
- Une fois qu'Ubuntu est démarré depuis la clé USB, suivre les instructions et installer Ubuntu 22.04 sur le PC en configuration minimale (en simple ou dual-boot selon l'utilisation souhaitée) ;
- A la fin de l'installation, connecter le PC à Internet, puis lancer l'application "Software Updater" ;
- Installer les mises à jour les plus récentes et redémarrer le PC ;
- Dernière étape : ouvrir un terminal de commande et entrer les commandes suivantes : ```sudo apt-get update```, puis ```sudo apt-get upgrade```.

### Configuration de Python
Une version de Python 3.10 est livrée et installée par défaut sur Ubuntu 22.04, aucune autre installation de Python n'est nécessaire.
En revanche, le gestionnaire de paquets pip n'est pas installé par défaut.
- Installer pip pour Python3 avec la commande ```sudo apt install python3-pip```.

Pour séparer les projets Python dans des environnements distancts (et afin d'éviter les problèmes de compatibilité entre les librairies utilisées par différents projets,
il est préferrable d'isoler chaque projet dans un environnement virtuel qui contient uniquement les librairies Python nécessaiers au projet.
Le module "venv" permet de créer et gérer les environnements virtuels.
- Installer venv avec la commande ```sudo apt install python3.10-venv```.

### Installation du cfclient
Le cfclient est l'application officielle de Bitcraze pour configurer et faire voler les drones Crazyflie. 
Pour ce projet, il ne sera utilisé que pour configurer les drones.
La procédure d'installation du cfclient est rappelée ici, bien qu'elle soit disponible sur le [site officiel de Bitcraze]([https://pages.github.com/](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/))
- Créer un environnement virtuel spécifique au cfclient avec la commande ```python3 -m venv cfclient```. Cette commande crée un dossier "cfclient" dans le répertoire ```~/``` (Home), dans lequel un nouvel interpréteur Python 3.10 est ajouté.
- Activer l'environnement virtuel avec la commande ```source cfclient/bin/activate```.
- Installer les pré-requis du cfclient dans son environnement virtuel avec les commandes suivantes :
  + ```sudo apt install git```
  + ```sudo apt insall python3-pip```
  + ```sudo apt install libxcb-xinerama0```
  + ```sudo pip3 install --upgrade pip```

- Autoriser l'utilisation des Dongle USB "CrazyRadioPA" sans les permissions d'administrateur :
  + ```sudo groupadd plugdev```
  + ```sudo usermod -a -G plugdev $USER```
  + Copier-coller du texte suivant en tant que commande : 
```  
cat <<EOF | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
```

- Installer le cfclient avec la commande suivante : ```pip3 install cfclient```
- Une fois installé, le cfclient peut être exécuté à partir de son environnement virtuel avec la commande ```cfclient```.
- Quitter l'environnement virtuel avec la commande ```deactivate```.

### Instalation de PyCharm
- Installer PyCharm Community Edition sia le gestionnaire de paquets "snap" en utilisant la commande : ```sudo snap install pycharm-community --classic```.

### Installation de JupyterLab
-Installer JupyterLab avec la commande : ``` ```.

### Création des raccourcis
La procédure de lancement du cfclient n'est pas très "user-friendly" actuellement : il faut ouvrir un terminal de commande, activer le bon environnement 
virtuel, puis taper la commande ```cfclient```.
Il en va de même pour JupyterLab. C'est pourquoi je propose d'automatiser ces procédures en créant un raccourci dans la liste des applications 
pour lancer ces programmes en un clic.
- Créer un nouveau fichier sur le bureau à l'aide de l'éditeur de texte. Le nommer "cfclient.desktop".
- Télécharger le logo de Bitcraze et l'enregistrer sur le PC.
- Copier-coller le texte suivant dans ce fichier et l'enregistrer (modifier les chemins d'accès si besoin).
```
[Desktop Entry]
Version=2022.12
Name=cfclient
Comment=Official Bitcraze client to fly the Crazyflie.
Exec=/home/eae/cfclient/bin/cfclient
Icon=/home/eae/cfclient/Bitcraze_logo.png
Terminal=true
Type=Application
Categories=Application
```

- Sur le bureau, faire un clic-droit sur ce fichier, puis "Properties", "Permissions", et "Allow executing file as a program".
- Faire un deuxième clic-droit sur ce fichier, puis "Allow launching".
Le fichier ainsi créé agit comme un raccourci sur le bureau : un double clic lance le cfclient.
Pour intégrer ce raccourci à la liste des applications, il est nécessaire de lancer l'explorateur de fichiers en tant qu'administrateur avec la commande 
```sudo nautilus```
- Faire un copier-coller du fichier "cfclient.desktop" dans le répertoire ```/usr/share/applications/``` (remonter le chemin d'accès jusqu'à la racine).
Le raccourci apparaît alors dans le menu des applications (touche Windows sur le bureau), et il est possible de l'ajouter à la barre des favoris 
(clic-droit, puis "add to favourites").

- Répéter la procédure avec JupyterLab.

