# 🚀 Projet Robot -- BFS & Génération d'Obstacles

Ce projet implémente un système permettant à un robot de se déplacer
dans une grille contenant des obstacles, tout en respectant des
contraintes géométriques liées à sa taille.\
L'objectif principal est de déterminer une **séquence minimale de
commandes** permettant au robot d'atteindre une cible grâce à un **BFS
spécialisé**.

Il inclut également des programmes d'expérimentation (performances du
BFS) et une interface utilisant **Gurobi** pour la génération optimisée
d'obstacles.

------------------------------------------------------------------------

## 📁 Contenu du projet

  -----------------------------------------------------------------------
  Fichier                                  Rôle
  ---------------------------------------- ------------------------------
  `robot.py`                               Algorithmes centraux : BFS,
                                           vérifications géométriques,
                                           parsing d'instances

  `experiences_Qc.py`                      Expérimentations pour analyser
                                           le temps selon la taille de la
                                           grille

  `experiences_Qd.py`                      Expérimentations pour analyser
                                           le temps selon le nombre
                                           d'obstacles

  `interface_gurobi_robot.py`              Interface utilisant Gurobi
                                           pour placer des obstacles de
                                           manière optimale
  -----------------------------------------------------------------------

------------------------------------------------------------------------

## 🤖 Fonctionnement du robot

Le robot évolue uniquement **sur les sommets** d'une grille de cases.

Un état est défini par :

    (i, j, orientation)

où `orientation ∈ {nord, est, sud, ouest}`.

### Commandes autorisées

-   `G` --- tourner à gauche\
-   `D` --- tourner à droite\
-   `a1` --- avancer de 1 sommet\
-   `a2` --- avancer de 2 sommets\
-   `a3` --- avancer de 3 sommets

### Contraintes géométriques

Le robot possède un *diamètre de 1.6 mètres*.

#### ✔ Un sommet (i, j) n'est valide que si les 4 cases autour sont libres.

#### ✔ Un rail est franchissable seulement si les 2 cases adjacentes sont libres.

------------------------------------------------------------------------

## 🔍 Algorithme BFS

Le BFS explore l'espace des états `(i, j, o)` en tenant compte :

-   des rotations,
-   des déplacements de 1 à 3 sommets,
-   des contraintes d'obstacles,
-   de la géométrie du robot.

L'algorithme retourne : - la **liste minimale de commandes**, ou -
`None` si aucun chemin n'est possible.

------------------------------------------------------------------------

## 🧪 Expérimentations

### 📌 Question C --- Influence de la taille N de la grille

Le fichier `experiences_Qc.py` :

-   génère des instances pour différentes tailles de grille (10, 20, 30,
    40, 50),
-   mesure le temps d'exécution du BFS,
-   produit un graphique **temps moyen vs N**.

### 📌 Question D --- Influence du nombre d'obstacles P

Le fichier `experiences_Qd.py` :

-   fixe une grille 20×20,
-   fait varier P (10, 20, 30, 40, 50),
-   calcule le temps moyen de BFS selon P.

------------------------------------------------------------------------

## 🧮 Interface Gurobi

Le fichier `interface_gurobi_robot.py` permet :

-   de générer une grille d'obstacles optimisée via un **programme
    linéaire Gurobi**,\
-   d'interdire certains motifs (ex. motif 101),
-   de respecter des contraintes de densité d'obstacles,
-   de tester directement le BFS sur la grille obtenue.

L'utilisateur peut ensuite saisir :

-   un sommet de départ,
-   un sommet d'arrivée,
-   une orientation initiale.

Le programme affiche la séquence minimale de commandes.

------------------------------------------------------------------------

## 🛠️ Dépendances

### Obligatoires

-   Python 3.x
-   `matplotlib`
-   `statistics`
-   `gurobipy` (uniquement pour l'interface Gurobi)

### Installation rapide

``` bash
pip install matplotlib gurobipy
```

------------------------------------------------------------------------

## ▶️ Utilisation

### Lancer le BFS sur un fichier d'instances

``` bash
Get-Content test.txt | python robot.py 
```

### Lancer les expériences question C

``` bash
python experiences_Qc.py
```

### Lancer les expériences question D

``` bash
python experiences_Qd.py
```

### Interface Gurobi

``` bash
python interface_gurobi_robot.py
```

------------------------------------------------------------------------

## 📄 Format d'une instance

    M N
    <grille de M lignes contenant 0 ou 1>
    D1 D2 F1 F2 orientation

Une instance finale `"0 0"` marque la fin du fichier.

------------------------------------------------------------------------

## 👤 Auteurs

Projet réalisé par **Djamel Salah** et **Naoufel AZIZI**.

------------------------------------------------------------------------