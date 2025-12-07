#!/usr/bin/env python3
import random
import argparse


ORIENTATIONS = ["nord", "sud", "est", "ouest"]


def generate_instance(M, N, P):
    """
    Génère une instance :
    - grille M x N avec P obstacles (1) et le reste à 0,
    - point de départ (D1, D2),
    - point d'arrivée (F1, F2),
    - orientation initiale.
    """
    if P > M * N - 2:
        raise ValueError("Trop d'obstacles : il faut laisser au moins 2 cases libres (départ et arrivée).")

    # Grille initiale remplie de 0
    grid = [[0 for _ in range(N)] for _ in range(M)]

    # Tirage aléatoire du départ et de l'arrivée
    D1, D2 = random.randrange(M), random.randrange(N)
    while True:
        F1, F2 = random.randrange(M), random.randrange(N)
        if (F1, F2) != (D1, D2):
            break

    # Placement des obstacles (tous différents, pas sur départ ni arrivée)
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

    # Orientation aléatoire
    orientation = random.choice(ORIENTATIONS)

    return grid, (D1, D2), (F1, F2), orientation


def print_instance(M, N, grid, start, end, orientation):
    """
    Affiche une instance au format demandé :
    M N
    <M lignes de N nombres 0/1>
    D1 D2 F1 F2 orientation
    """
    D1, D2 = start
    F1, F2 = end

    # Première ligne : dimensions
    print(M, N)

    # Grille
    for i in range(M):
        print(" ".join(str(grid[i][j]) for j in range(N)))

    # Ligne de départ / arrivée / orientation
    print(D1, D2, F1, F2, orientation)


def main():
    parser = argparse.ArgumentParser(
        description="Générateur d'instances pour le problème du robot (MOGPL)."
    )
    parser.add_argument("M", type=int, help="Nombre de lignes de la grille")
    parser.add_argument("N", type=int, help="Nombre de colonnes de la grille")
    parser.add_argument("P", type=int, help="Nombre d'obstacles")
    parser.add_argument(
        "-k", "--instances", type=int, default=1,
        help="Nombre d'instances à générer (par défaut 1)"
    )
    parser.add_argument(
        "--seed", type=int, default=None,
        help="Graine aléatoire (optionnelle, pour reproductibilité)"
    )

    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    M, N, P = args.M, args.N, args.P

    for _ in range(args.instances):
        grid, start, end, orientation = generate_instance(M, N, P)
        print_instance(M, N, grid, start, end, orientation)

    # Bloc final de fin de fichier comme dans l'énoncé
    print("0 0")


if __name__ == "__main__":
    main()
