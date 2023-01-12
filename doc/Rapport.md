# Rapport

## Membre de l’équipe
- Carlens Belony
- Jean-Christophe Caron
- Maxence Guindon
- Samuel Horvat

## Libraire FiniteStateMachine
### Pourcentage 100 %
En tant qu’équipes, nous avons suivi le UML fourni à la lettre pour réussir à faire un FiniteStateMachine qui soit le plus fidèle possible à la conception fourni. Il se peut que des erreurs de compréhension subsistent lors de la remise, mais nous avons fait de notre mieux pour discuter avec le professeur pour être certains de bien comprendre son *design* et éliminer le plus d’erreurs possible.

## Classe Robot
### Validation de l’intégrité du robot
Pour valider l’intégrité du robot, nous avons instancié les 4 éléments externes à celui-ci. Soit la télécommande, les servos de la caméra et du télémètre ainsi que le télémètre lui-même. Nous avons vérifié que l’instanciation ne retournait pas None, ainsi si l’un des quatre objets ne réussit pas son instanciation, nous fermons le robot.


### Gestion de la télécommande
Pour la télécommande, nous avons tenté de faire le système de bascule, mais finalement, nous avons opté pour changer le bouton pour quitter l’application par «*» plutôt que par «ok». La télécommande est instanciée par la classe Robot même si elle est une classe en elle-même.

### Gestion du télémètre et de son servo moteur
Pour la gestion du télémètre, nous avons la lecture de distance dans une fonction appelée read_distance qui retourne la distance selon la mesure sélectionnée. La mesure de base est en millimètre.

### Gestion des moteurs
Pour les moteurs, nous avons encapsulé les actions de déplacements du robot dans quatre fonctions que nous avons appelées move_forwards, move_backwards, turn_left et turn_right. De plus, nous avons créé la fonction stop_moving qui arrête le robot.

### Gestion de la couleur pour les yeux (EyeBlinkers)
Pour la gestion de la couleur des yeux, nous avons une fonction appelée change_eyes_color. Cette fonction prend en paramètre, une couleur ainsi que l’œil ou les yeux sur lequel ou sur lesquels il s’applique. La fonction possède une fonction interne (clamp) qui s’assure que le tuple n’ait pas de valeur en dessous de 0 et au-dessus de 255.

## Structure générale du logiciel
Nous avons quatre documents qui forment le logiciel.
- Niveau1.py
- Niveau2.py
- Niveau3.py
- Application.py
- main. ipynb

Le niveau 1 contient la base de la librairie. Les classes FiniteStateMachine, Layout, Transition et State sont dans ce fichier.

Le niveau 2 contient toutes les classes qui font des abstractions sur les classes State et Transition (les classes Action et Monitered). De plus, le fichier contient les classes de conditions qui permettent à une transition de s’appliquer et de changer l’état du logiciel.

Le niveau 3 contient des classes plus avancées. Soit les classes Blinker et SideBlinker. Ceux-ci servent à allumer et éteindre les yeux et les phares.

Le fichier application contient les classes essentielles à l’interaction avec le robot. Notamment les classes EyeBlinkers, LedBlinkers, Robot et C64.

Le fichier main. ipynb est le fichier jupiter Notebook qui permet de lancer le robot.

Du point de vue applicatif, lorsqu’un robot est lancé, il y a plusieurs State Machine qui vont fonctionner simultanément. Tout d’abord, il y a celui de la classe C64 qui gère l’application, ensuite, il y a ceux des tâches 1 et 2 qui roulent ainsi que ceux des yeux et des phares qui peuvent aussi être activés par l’utilisateur.

### Capacité modulaire
Pour ajouter une nouvelle tâche à C64, il suffit de créer sa tâche et de l’ajouter à la liste des tâches et à ajouter les actions s’il y a lieu. Pour connecter ses transitions, on peut utiliser les fonctions qui sont en haut du fichier Application et qui s’acquittent de cette action.

## Autres abstraction?
Dans le fichier Application, nous avons créé la classe RobotState pour bien encapsuler toutes les fonctionnalités de la classe MonitoredState et nous permettre d’y ajouter les actions spécifiques au robot. Nous avons procédé ainsi pour d’autres procédés du projet comme pour des conditions (notamment les RemoteControlCondtion) et les tâches à faire qui héritait du FiniteStateMachine.

## Tâches 1 & 2
### Tâche 1
Pour la tâche 1, le mandat était de faire bouger le robot lorsqu’on appuie sur une des flèches de la manette. Le robot doit également faire clignoter ses yeux de manière asynchrone avec un œil de couleur bleu et l’autre de couleur rouge. Les phares clignotent selon les directions que prend le robot (avant, arrière, droite, gauche).

### Tâche 2
Pour la tâche 2, le robot clignote ses deux yeux en même temps avec une couleur violette. On indique ensuite une direction au robot, soit droite, soit gauche. Ensuite le robot va avancer de manière autonome jusqu’à ce qu’il rencontre un obstacle à 100 mm de lui grâce au télémètre. Le robot tournera alors dans la direction indiquée par l’utilisateur plus tôt en faisant un clin d’œil avec celui qui est dans la même direction que lui. Après 1,2 seconde de rotation, le robot repartira en ligne droite jusqu’au prochain obstacle.