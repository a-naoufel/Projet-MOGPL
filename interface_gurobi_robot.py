import random
import sys

try:
    import gurobipy as gp
    from gurobipy import GRB
except ImportError:
    print("Erreur : le module gurobipy n'est pas disponible.")
    sys.exit(1)

import robot  


def build_obstacle_grid_with_gurobi(M, N, P):
    """
    Construit et résout le PL de placement des P obstacles sur une grille de cases MxN
    (indices de cases 0..M-1, 0..N-1), en minimisant la somme des poids w_ij, sous
    les contraintes énoncées dans le sujet.
    """
    # Génération des poids aléatoires w_ij
    weights = [[random.randint(0, 1000) for _ in range(N)] for _ in range(M)]

    # Création du modèle Gurobi
    model = gp.Model("obstacle_placement")

    # Variables x_ij binaires pour les cases (i,j), i=0..M-1, j=0..N-1
    x = model.addVars(
        range(M), range(N),
        vtype=GRB.BINARY,
        name="x"
    )

    # Fonction objectif : min sum w_ij * x_ij
    model.setObjective(
        gp.quicksum(weights[i][j] * x[i, j] for i in range(M) for j in range(N)),
        GRB.MINIMIZE
    )

    #Contrainte : exactement P obstacles
    model.addConstr(
        gp.quicksum(x[i, j] for i in range(M) for j in range(N)) == P,
        name="total_obstacles"
    )

    # Contrainte : au plus 2P/M obstacles par ligne
    max_per_row = 2 * P / M
    for i in range(M):
        model.addConstr(
            gp.quicksum(x[i, j] for j in range(N)) <= max_per_row,
            name=f"row_{i}"
        )

    #Contrainte : au plus 2P/N obstacles par colonne
    max_per_col = 2 * P / N
    for j in range(N):
        model.addConstr(
            gp.quicksum(x[i, j] for i in range(M)) <= max_per_col,
            name=f"col_{j}"
        )

    #Interdiction du motif 101 sur les lignes :
    #    pour chaque ligne i, chaque triplet j, j+1, j+2
    for i in range(M):
        for j in range(N - 2):
            model.addConstr(
                x[i, j] + x[i, j + 2] <= 1 + x[i, j + 1],
                name=f"no_101_row_{i}_{j}"
            )

    # Interdiction du motif 101 sur les colonnes :
    for i in range(M - 2):
        for j in range(N):
            model.addConstr(
                x[i, j] + x[i + 2, j] <= 1 + x[i + 1, j],
                name=f"no_101_col_{i}_{j}"
            )

    #Résolution
    model.setParam("OutputFlag", 0)  
    model.optimize()

    if model.status != GRB.OPTIMAL:
        print("Pas de solution optimale trouvée (status Gurobi =", model.status, ")")
        return None, None

    # Construction de la grille de cases obstacle (M x N)
    grid = [[0 for _ in range(N)] for _ in range(M)]
    for i in range(M):
        for j in range(N):
            val = x[i, j].X
            grid[i][j] = 1 if val > 0.5 else 0

    return grid, weights


def print_grid(grid):
    """Affiche la grille 0/1 des CASES de façon lisible."""
    M = len(grid)
    N = len(grid[0])
    print("\nGrille d'obstacles (1 = obstacle, 0 = libre) :")
    for i in range(M):
        print(" ".join(str(grid[i][j]) for j in range(N)))


def main():
    print("Interface Gurobi + BFS")

    # Choix de M, N, P
    try:
        M = int(input("Nombre de lignes de cases M: "))
        N = int(input("Nombre de colonnes de cases N: "))
        P = int(input("Nombre d'obstacles P: "))
    except ValueError:
        print("Entrée invalide, il faut des entiers.")
        return

    if P <= 0 or P >= M * N:
        print("Nombre d'obstacles P incohérent.")
        return

    if M < 3 or N < 3:
        print("La grille doit être au moins 3x3 pour que les sommets intérieurs existent.")
        return

    #Génération des obstacles par PL + Gurobi
    grid, weights = build_obstacle_grid_with_gurobi(M, N, P)
    if grid is None:
        print("Impossible de générer la grille d'obstacles.")
        return

    print_grid(grid)


    try:
        D1 = int(input(f"D1 (ligne du sommet de départ, entre 1 et {M-1}) : "))
        D2 = int(input(f"D2 (colonne du sommet de départ, entre 1 et {N-1}) : "))
        F1 = int(input(f"F1 (ligne du sommet d'arrivée, entre 1 et {M-1}) : "))
        F2 = int(input(f"F2 (colonne du sommet d'arrivée, entre 1 et {N-1}) : "))
    except ValueError:
        print("Entrée invalide.")
        return

    # Vérification du domaine des sommets
    if not (1 <= D1 <= M - 1 and 1 <= D2 <= N - 1 and
            1 <= F1 <= M - 1 and 1 <= F2 <= N - 1):
        print("Les sommets doivent être strictement intérieurs (entre 1 et M-1 / 1 et N-1).")
        return

    # Vérification géométrique via vertex_ok
    if not robot.vertex_ok(D1, D2, grid):
        print("Le sommet de départ n'est pas valide pour le robot")
        return
    if not robot.vertex_ok(F1, F2, grid):
        print("Le sommet d'arrivée n'est pas valide pour le robot")
        return

    ori_str = input("Orientation initiale (nord/est/sud/ouest) : ").strip().lower()
    if ori_str not in robot.ORI_STR_TO_ID:
        print("Orientation invalide.")
        return
    start_o = robot.ORI_STR_TO_ID[ori_str]

    # lancement de bfs depuis le fichier robot.py
    cmds = robot.bfs(grid, D1, D2, start_o, F1, F2)

    if cmds is None:
        print("Aucun chemin trouvé par l'algorithme (bfs a renvoyé None).")
    else:
        print("Nombre de commandes :", len(cmds))
        print("Séquence de commandes :")
        print(" ".join(cmds))


if __name__ == "__main__":
    main()
