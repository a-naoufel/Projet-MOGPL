#!/usr/bin/env python3
import random
import time
import matplotlib.pyplot as plt
from statistics import mean, stdev

import robot 

ORIENTATIONS = ["nord", "sud", "est", "ouest"]


def generate_instance(M, N, P):
    """
    Génère UNE instance :
      - grille MxN avec P obstacles,
      - départ (D1,D2), arrivée (F1,F2),
      - orientation initiale.
    On impose ici que départ et arrivée soient strictement à l'intérieur :
      1 <= D1 <= M-2, 1 <= D2 <= N-2, idem pour F1,F2.
    """
    if P > M * N - 2:
        raise ValueError("Trop d'obstacles : il faut laisser au moins 2 cases libres.")

    if M < 3 or N < 3:
        raise ValueError("La grille doit être au moins 3x3.")

    # Grille pleine de zéros
    grid = [[0 for _ in range(N)] for _ in range(M)]

    # Départ à l'intérieur
    D1 = random.randrange(1, M - 1)
    D2 = random.randrange(1, N - 1)

    # Arrivée à l'intérieur, différente du départ
    while True:
        F1 = random.randrange(1, M - 1)
        F2 = random.randrange(1, N - 1)
        if (F1, F2) != (D1, D2):
            break

    # Obstacles : P cases distinctes, pas sur départ/arrivée
    obstacles = set()
    while len(obstacles) < P:
        i = random.randrange(M)
        j = random.randrange(N)
        if (i, j) in obstacles:
            continue
        if (i, j) == (D1, D2) or (i, j) == (F1, F2):
            continue
        obstacles.add((i, j))

    for (i, j) in obstacles:
        grid[i][j] = 1

    orientation = random.choice(ORIENTATIONS)
    return M, N, grid, (D1, D2), (F1, F2), orientation


def write_instance_block(f, M, N, grid, start, end, orientation):
    """
    Écrit UNE instance dans le fichier d'entrée, SANS la ligne terminale "0 0".
    Format :
      M N
      <M lignes de N entiers 0/1>
      D1 D2 F1 F2 orientation
    """
    D1, D2 = start
    F1, F2 = end

    # Ligne M N
    f.write(f"{M} {N}\n")

    # Grille
    for i in range(M):
        f.write(" ".join(str(grid[i][j]) for j in range(N)) + "\n")

    # Ligne départ / arrivée / orientation
    f.write(f"{D1} {D2} {F1} {F2} {orientation}\n")


def main():
    sizes = [10, 20, 30, 40, 50]
    nb_instances = 10

    random.seed()  

    # Fichiers demandés par l'énoncé
    entree_filename = "entree_Qc.txt"
    resultats_filename = "resultats_Qc.txt"

    # Pour stocker les temps par taille de grille
    times_by_size = {N: [] for N in sizes}

    with open(entree_filename, "w") as f_in, open(resultats_filename, "w") as f_out:
        for size in sizes:
            M = N = size
            P = size  # nombre d'obstacles = taille

            for k in range(nb_instances):
                # 1) Génération d'une instance
                M, N, grid, start, end, orientation = generate_instance(M, N, P)

                # 2) Écriture de l'instance dans le fichier d'entrée
                write_instance_block(f_in, M, N, grid, start, end, orientation)

                # 3) Lancement de ton BFS et mesure du temps
                D1, D2 = start
                F1, F2 = end
                start_o = robot.ORI_STR_TO_ID[orientation]

                t0 = time.perf_counter()
                cmds = robot.bfs(grid, D1, D2, start_o, F1, F2)
                t1 = time.perf_counter()

                dt = t1 - t0
                times_by_size[size].append(dt)

                # 4) Écriture de la ligne de résultat correspondante
                if cmds is None:
                    f_out.write("-1\n")
                else:
                    T = len(cmds)
                    f_out.write(str(T) + " " + " ".join(cmds) + "\n")

        # 5) Ligne finale "0 0" pour terminer le fichier d'entrée
        f_in.write("0 0\n")

    # Affichage des temps moyens (en ms) pour le rapport
    print("\n=== Temps moyens d'exécution (question c) ===")
    print(f"{'N':>5}  {'moyen (ms)':>12}  {'écart-type (ms)':>16}")
    for N in sizes:
        temps = times_by_size[N]
        moy_ms = mean(temps) * 1000
        if len(temps) > 1:
            std_ms = stdev(temps) * 1000
        else:
            std_ms = 0.0
        print(f"{N:5d}  {moy_ms:12.3f}  {std_ms:16.3f}")

        Ns = sorted(times_by_size.keys())
    moyennes_ms = []
    ecarts_ms = []

    for N in Ns:
        temps = times_by_size[N]
        moyennes_ms.append(mean(temps) * 1000)
        if len(temps) > 1:
            ecarts_ms.append(stdev(temps) * 1000)
        else:
            ecarts_ms.append(0.0)

    plt.figure()
    plt.errorbar(Ns, moyennes_ms, yerr=ecarts_ms, fmt='-o', capsize=5)
    plt.title("Temps moyen d'exécution BFS selon la taille de la grille N")
    plt.xlabel("Taille de la grille N")
    plt.ylabel("Temps moyen (ms)")
    plt.grid(True)
    plt.show()

    print(f"\nInstances écrites dans : {entree_filename}")
    print(f"Résultats écrits dans : {resultats_filename}")


if __name__ == "__main__":
    main()
