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

## Modélisation

Le comportement du dispositif est modélisé par une équation dynamique, avec une commande qui est la force $F$ qui s'exerce sur le chariot. La fonction **modelependule** associée est présente dans le schéma Simulink.

```
function etatp = modelependule(etat,u,m,M,g,l)
theta=etat(2);xp=etat(3);thetap=etat(4);
F=u;

xpp=(F+m*g*sin(theta)*cos(theta)-m*l*thetap^2*sin(theta))/(M+m*sin(theta)^2);
thetapp=(xpp*cos(theta)+g*sin(theta))/l;

etatp=zeros(size(etat));
etatp(1)=xp;
etatp(2)=thetap;
etatp(3)=xpp;
etatp(4)=thetapp;
```

Le modèle est complété par l'équation qui lie la tension $u_m$ d'alimentation du moteur et la force $F$ qui en résulte. 

$$F=k_1 u_m + k_2 \dot x$$

## Réglages de l'Asservissement

Le point de départ est le modèle linéarisé tangent.
```
% Modélisation du système
M=1;m=0.1;l=0.5;g=9.81;
k1=20;k2=100;
xinit=0.0;thetainit=0.0;

% Modèle d'état du système linéarisé tangent
A=[0 0 1 0;0 0 0 1;0 m*g/M -k2/M 0;0 (M+m)*g/(M*l) -k2/(M*l) 0];
B=[0; 0; k1/M; k1/(M*l)];
C=[1 0 0 0;0 1 0 0];
D=zeros(2,1);
```

On envisage de faire appel à une action intégrale, ce qui demande de considérer un modèle étendu.
```
% modèle étendu (x,theta,xp,thetap,q)
% q est l'intégration de x
Ai=[A zeros(4,1);1 0 0 0 0];Bi=[B;0];
```

Sur la base de ce modèle étendu, on règle le retour d'état.
```
% retour d'état LQR
QXU=diag([1e3 1e3 10 10 1e4 1]);
Ki=lqr(Ai,Bi,QXU(1:5,1:5),QXU(6,6));
```

L'observateur d'état ne s'applique en revanche que sur les variables d'état du modèle initial.
```
% observateur
QWV=diag([100 100 100 100 1 1]);
pendulek=ss(A,[B eye(4)],C,zeros(2,5));
[~,L] = kalman(pendulek,QWV(1:4,1:4),QWV(5:6,5:6));
Aobs=A-L*C;Bobs=[B L];Cobs=eye(4);Dobs=zeros(4,3);
```

## La vidéo

Après lancement de la simulation, on obtient la restitution suivante.

https://github.com/user-attachments/assets/284c3366-8a48-45d8-9631-04f487a2d021
