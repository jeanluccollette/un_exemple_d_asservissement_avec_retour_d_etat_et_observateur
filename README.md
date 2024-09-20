# Asservissement appliqué à un pendule inversé sur chariot 

## Description

La simulation d'un pendule inversé sur chariot est proposée, avec une animation 3D. Elle fait appel à Simulink dans Matlab.

## Configuration

Les toolboxes nécessaires dans Matlab sont les suivants :
- Simulink
- Control System Toolbox
- Simulink 3D Animation

## Les fichiers

Les fichiers permettant la simulation sont dans le [fichier archive](chariot_et_pendule.zip) joint.

- **pendule_chariot.pdf** : description du modèle
- **pendule_lineaire_haut.wrl**, **logocs.png**, **signature.png** : fichiers pour l'animation 3D
- **pendule_chariot_lqg_prtrb_int_ini.m** : script pour l'initialisation des variables dans le schéma
- **pendule_chariot_lqg_prtrb_int_sim.slx**, **pendule_chariot_lqg_prtrb_int_sim_r2019b.slx** : schéma Simulink (avec version r2019b au besoin)

## La vidéo

Après lancement de la simulation, on obtient la restitution suivante.

![](chariot_et_pendule.mp4)