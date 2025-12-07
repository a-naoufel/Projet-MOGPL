#!/usr/bin/env python3
import random
import os

ORIENTATIONS = ["nord", "sud", "est", "ouest"]


def generate_instance(M, N, P):
    """Génère une instance MxN avec P obstacles, départ, arrivée et orientation."""
    if P > M * N - 2:
        raise ValueError("Trop d'obstacles : il faut laisser 2 cases libres.")

    grid = [[0 for _ in range(N)] for _ in range(M)]

    # Tirage du point de départ
    D1, D2 = random.randrange(M), random.randrange(N)

    # Tirage du point d'arrivée différent du départ
    while True:
        F1, F2 = random.randrange(M), random.randrange(N)
        if (F1, F2) != (D1, D2):
            break

    # Placement des obstacles
    obstacles = set()
    while len(obstacles) < P:
        i = random.randrange(M)
        j = random.randrange(N)
        if (i, j) not in obstacles and (i, j) != (D1, D2) and (i, j) != (F1, F2):
            obstacles.add((i, j))

    for (i, j) in obstacles:
        grid[i][j] = 1

    orientation = random.choice(ORIENTATIONS)
    return grid, (D1, D2), (F1, F2), orientation


def write_instance(filename, M, N, grid, start, end, orientation):
    """Écrit une seule instance dans un fichier."""
    D1, D2 = start
    F1, F2 = end

    with open(filename, "w") as f:
        f.write(f"{M} {N}\n")

        for i in range(M):
            f.write(" ".join(str(grid[i][j]) for j in range(N)) + "\n")

        f.write(f"{D1} {D2} {F1} {F2} {orientation}\n")

        # Ligne finale obligatoire
        f.write("0 0\n")


def generate_all():
    sizes = [10, 20, 30, 40, 50]

    for size in sizes:
        M = N = 20
        P = size  # nombre d'obstacles = taille
        # os.makedirs(f"instances_{size}", exist_ok=True)

        # print(f"Génération de 10 instances {size}x{size} dans dossier instances_{size}/ ...")

        for i in range(1):  # 10 instances par taille
            grid, start, end, ori = generate_instance(M, N, P)
            # filename = f"instances_{size}/instance_{size}_{i+1}.txt"
            filename = f"instance_{size}_{i+1}d.txt"
            write_instance(filename, M, N, grid, start, end, ori)

        print(f"✔ Dossier instances_{size} complété.")


if __name__ == "__main__":
    generate_all()
