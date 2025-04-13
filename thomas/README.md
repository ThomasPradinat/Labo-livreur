Voici le package ROS contenant le code de mon projet

# Les programmes dans le dossier src sont à explorer dans l'ordre suivant : 
## 1. send_goal.cpp :
Ce node a pour but de communiquer avec MoveBase. Le fait d'utiliser un client est plus approprié qu'un simple publisher car il permet aussi de recevoir des informations sur l'avancement des mouvements du robot.

## 2. multi_goal.cpp
Ce programme a pour but d'envoyer différents goal à MoveBase, comme le programme send_goal.cpp
Les coordonnées des goals a envoyé sont lues depuis un fichier goals.yaml. Chaque coordonée a un identifiant.
Un autre fichier route.yaml contient une liste d'identifiant. Ce sont ceux des points que l'on souhaite envoyé. 
Le programme stocke donc les coordonnées de goals.yaml, puis lie dans l'ordre les identifiant du fichier route.yaml et envoie les coordonnées correspondantes à MoveBase

## 3. path_finding.cpp
Ce programme de path-finding utilise la méthode du "travel salesman problem" (tsp) :
	il s'agit de créer une boucle passant par tous les points en parcourant la plus petite distance possible
Les points d'un graphe sont inscrit dans un fichier noeuds.yaml
Les points que l'on veut relié avec le tsp sont écrit dans le fichier objectifs.yaml
Ce programme lie les deux fichiers, utilise le graphe pour créer une matrice de distances entre les différents objectifs souhaités puis le tsp renvoie l'ordre des points qu'il faut suivre pour parcourir la plus petite distance
Le chemin final est envoyé dans un topic "tsp_path" pour être utiliser plus tard

## 4. path_listener.cpp
Ce programme a été créé pour tester le programme path-finding.cpp
Il lie le contenu du topic "tsp_path", c'est un simple subscriber

## 5. path_maker.cpp
Ce programme est un mélange de multi_goal.cpp et de path_listener.cpp
Il envoie différents goals à MoveBase en prennant en référence les coordonnées entrées dans goals.yaml mais la liste des goals a envoyé ne vient plus d'un fichier .yaml mais est lue dans le topic tsp_path envoyé par path_finding.cpp

# Les fichier .launch sont à découvrir dans l'ordre suivant : 
## 1. sans rien
Il faut avant tout qu'un programme de navigation Rviz soit en cour, car les neuds vont tenter de se lier à lui.
Le programme send_goal.cpp sert à vérifier ce lien entre les programme, lancez le seul pour voir si la fenêtre Rviz prend bien en compte le goal envoyé par send_goal.cpp
Tout les autres programmes devront aussi être lancé en parallèle d'un programme de navigation.
## 2. multi_goal
Ce fichier permet de lancer multi_goal.cpp en prenant en compte les fichier .yaml dans lequel il va chercher ses instructions.
goals.yaml défini des points sur la carte avec des coordonnées et leur identifiant, il faut éviter de changer ce fichier
route.yaml indique une suite d'identifiant à envoyé au programme, ce fichier peut etre modifier pour tester différentes suites facilement.
## 3. optimal_path
Ce fichier permet de lancer simultanément path_finding.cpp et path_maker.cpp
path_maker.cpp utilise goals.yaml
goals.yaml défini des points sur la carte avec des coordonnées et leur identifiant, il faut éviter de changer ce fichier
path_finder utilise noeuds.yaml et objectifs.yaml.
noeuds.yaml défini les différentes coordonnées et liaisons des points du graphe. Il ne doit pas être modifié.
objectifs.yaml indique les points par lesquels l'utilisateur souhaite passer, ce fichier peut être changer au bon vouloir de l'utilisateur.
