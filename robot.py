import sys
from collections import deque

# Mapping orientations <-> indices
ORI_STR_TO_ID = {
    "nord": 0,
    "est": 1,
    "sud": 2,
    "ouest": 3,
}
ORI_ID_TO_STR = ["nord", "est", "sud", "ouest"]

# Vecteurs de déplacement (nord, est, sud, ouest)
DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)]


def vertex_ok(i, j, grid):
    """
    Vérifie si le sommet (i, j) est "dégagé" pour le robot.
    Le robot a un diamètre 1, donc les 4 cases autour du sommet
    ne doivent pas être des obstacles.

    Les 4 cases potentielles autour de (i, j) sont :
      (i-1, j-1), (i-1, j), (i, j-1), (i, j)
    Les cases hors de la grille sont considérées comme libres.
    """
    M = len(grid)
    N = len(grid[0])

    # Interdiction de tous les sommets frontières
    if i == 0 or j == 0 or i == M or j == N:
        return False

    # Vérifier les 4 cases autour du sommet
    for di in (-1, 0):
        for dj in (-1, 0):
            r = i + di
            c = j + dj
            if 0 <= r < M and 0 <= c < N:
                if grid[r][c] == 1:
                    return False
    return True

def edge_ok(i, j, di, dj, grid):
    """
    Vérifie que le rail entre (i, j) et (i+di, j+dj) est franchissable :
    les deux cases adjacentes à ce rail doivent être libres (ou hors de la grille).

    - Pour un rail horizontal (di = 0), on regarde les cases au-dessus et en dessous.
    - Pour un rail vertical   (dj = 0), on regarde les cases à gauche et à droite.
    """
    M = len(grid)
    N = len(grid[0])

    i2 = i + di
    j2 = j + dj

    # Rail horizontal
    if di == 0:
        j_left = min(j, j2)
        # Cases au-dessus et en dessous du rail
        cells = [(i - 1, j_left), (i, j_left)]

    # Rail vertical
    else:
        i_up = min(i, i2)
        # Cases à gauche et à droite du rail
        cells = [(i_up, j - 1), (i_up, j)]

    for r, c in cells:
        if 0 <= r < M and 0 <= c < N:
            if grid[r][c] == 1:
                return False
    return True


def bfs(grid, start_i, start_j, start_o, goal_i, goal_j):
    """
    BFS sur l'espace des états (i, j, o) où (i, j) est un sommet de la grille de rails :
      i ∈ [0..M], j ∈ [0..N]
    o est l'orientation (0: nord, 1: est, 2: sud, 3: ouest).

    Le robot :
      - tourne sur place (G, D),
      - avance de 1, 2 ou 3 rails (a1, a2, a3),
      - ne peut emprunter un rail que si les 2 cases adjacentes sont libres,
      - ne peut occuper qu’un sommet entouré de 4 cases libres (diamètre 1).

    Retourne la séquence minimale de commandes sous forme de liste de chaînes
    ['D', 'a1', 'a3', ...] ou None s'il n'y a pas de chemin.
    """
    M = len(grid) # nombre de lignes de cases
    N = len(grid[0]) # nombre de colonnes de cases
    max_i = M          # sommets en i: 0..M
    max_j = N          # sommets en j: 0..N

    # On vérifie que les coordonnées de départ / arrivée sont dans [0..M]x[0..N]
    if not (0 <= start_i <= max_i and 0 <= start_j <= max_j):
        return None
    if not (0 <= goal_i <= max_i and 0 <= goal_j <= max_j):
        return None
    

    # On vérifie que les sommets de départ et d'arrivée sont "ok"
    if not vertex_ok(start_i, start_j, grid):
        return None
    if not vertex_ok(goal_i, goal_j, grid):
        return None


    # visited[i][j][o] = bool (liste de liste de liste de booléens , cad en 3D : M x N x 4 orientations)
    visited = [[[False] * 4 for _ in range(max_j + 1)] for _ in range(max_i + 1)]
    # parent[i][j][o] = (pi, pj, po, cmd) ou None si pas de parent (Mémorise l’état précédent dans le BFS et la commande utilisée afin de pouvoir reconstruire le chemin de et la séquence de commandes après avoir trouvé la solution)
    parent = [[[None] * 4 for _ in range(max_j + 1)] for _ in range(max_i + 1)]

    q = deque() #créé une file (pour utiliser FIFO), sert a explorer les etats par couches , d’abord la distance 0, puis distance 1, puis distance 2, etc.
    q.append((start_i, start_j, start_o)) #ajoute l'état initial à la file
    visited[start_i][start_j][start_o] = True #marque l'état initial comme visité

    found_state = None #variable pour stocker l'état final si trouvé

    while q: #Tant que la file n’est pas vide, on prend le prochain état.
        i, j, o = q.popleft() # retire et renvoie le premier élément de la file (l’état courant), position et orientation actuelles du robot que le BFS explore pour générer les états suivants.

        # Condition d'arrivée : bonne case, orientation quelconque, on a trouvé la case cible car d'après le principe du BFS, premier état trouvé = chemin le plus court
        if i == goal_i and j == goal_j:
            found_state = (i, j, o)
            break

        # Rotations gauche et droite
        for cmd, delta_o in (("G", -1), ("D", 1)):
            o2 = (o + delta_o) % 4 # nouvelle orientation après rotation, modulo 4 pour rester dans les indices valides cad entre 0 et 3
            if not visited[i][j][o2]:
                visited[i][j][o2] = True
                parent[i][j][o2] = (i, j, o, cmd)
                q.append((i, j, o2)) #à partir de position (i,j), on crée 2 nouveaux états correspondant aux 2 orientations possibles après rotation.

        # Avances de 1, 2, 3 cases
        di, dj = DIRS[o] # vecteur de déplacement selon l'orientation actuelle
        for n in (3, 2, 1): #on utilise cet ordre pour prioriser les grandes avancées
            ok = True
            ii, jj = i, j
            # On vérifie chaque case traversée si on reste dans la grille et si pour chacune d'elles il n'y a pas d'obstacle
            for step in range(1, n + 1):
                ni = ii + di
                nj = jj + dj

                # Rester dans les sommets [0..M] x [0..N]
                if not (0 <= ni <= max_i and 0 <= nj <= max_j):
                    ok = False
                    break

                # Le rail entre (ii, jj) et (ni, nj) doit être libre
                if not edge_ok(ii, jj, di, dj, grid):
                    ok = False
                    break

                # Le sommet d'arrivée doit être dégagé
                if not vertex_ok(ni, nj, grid):
                    ok = False
                    break

                ii, jj = ni, nj

            if not ok:
                continue


            # On peut avancer de n cases
            if not visited[ii][jj][o]:
                visited[ii][jj][o] = True
                parent[ii][jj][o] = (i, j, o, f"a{n}")
                q.append((ii, jj, o)) #état atteint après avoir avancé de n cases

    #si aucun état final n'a été trouvé, on retourne None
    if found_state is None:
        return None

    # Reconstruction du chemin de commandes
    ci, cj, co = found_state #état final trouvé par le BFS
    commands = []
    while True:
        #chaque état sait d'ou il vient et par quelle commande on est arrivé à lui 
        p = parent[ci][cj][co]
        if p is None: #seul l'état initial n'a pas de parent
            break
        pi, pj, po, cmd = p
        commands.append(cmd)
        ci, cj, co = pi, pj, po

    commands.reverse() #on fait ici un revers de la liste des commandes car on les a collectées en partant de l'état final vers l'état initial
    return commands


def solve(instances):
    """
    instances : liste de tuples (M, N, grid, D1, D2, F1, F2, ori_str)
    Retourne les lignes de sortie sous forme de liste de chaînes.
    """
    outputs = []
    for M, N, grid, D1, D2, F1, F2, ori_str in instances:
        start_o = ORI_STR_TO_ID[ori_str]
        cmds = bfs(grid, D1, D2, start_o, F1, F2)
        if cmds is None:
            outputs.append("-1")
        else:
            T = len(cmds)
            outputs.append(str(T) + " " + " ".join(cmds))
    return outputs


def read_input():
    """
    Lit toutes les instances depuis stdin.
    Retourne une liste d'instances.
    """
    lines = [line.strip() for line in sys.stdin if line.strip() != ""] #lit toutes les lignes non vides depuis l'entrée standard, lines est donc une liste de toutes les lignes utiles
    idx = 0
    instances = []

    while idx < len(lines):
        parts = lines[idx].split() #lit la ligne courante et la divise en parties
        idx += 1
        if len(parts) < 2: #si la ligne ne contient pas au moins 2 parties, on l'ignore et on passe à la suivante
            continue
        M, N = map(int, parts[:2]) #convertit les 2 premières parties en entiers M et N
        if M == 0 and N == 0: #condition d'arrêt : dernière ligne avec 0 0
            break

        grid = []
        #on va lire M lignes pour construire la grille
        for _ in range(M):
            row = list(map(int, lines[idx].split()))
            idx += 1
            grid.append(row)

        # Ligne de départ, arrivée et orientation
        parts = lines[idx].split()
        idx += 1
        D1, D2, F1, F2 = map(int, parts[:4])
        ori_str = parts[4].lower()

        instances.append((M, N, grid, D1, D2, F1, F2, ori_str)) # l'instance complète est ajoutée à la liste

    return instances


def main():
    instances = read_input()
    outputs = solve(instances)
    for line in outputs:
        print(line)


if __name__ == "__main__":
    main()
