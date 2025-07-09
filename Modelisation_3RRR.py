import pygame
import numpy as np
import sys
import math

# On definit les parametres mécaniques 
L1 = 100
L2 = 100
Rb = 132.2594
Re = 35
elbow = -1

# On definit les couleurs 
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PINK = (255, 105, 180)
WHITE = (255, 255, 255)
YELLOW = (255, 255, 0)

# On initialise la fenetre pygame
pygame.init()
font = pygame.font.SysFont('Arial', 24)
size = (800, 800)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Simulation Robot 3RRR")
clock = pygame.time.Clock()

# On initialise la position de départ 
xE, yE, thetaE = 0, 0, 0
trajectory = []
prev_q = None

# On fixe le mode automatique (dessin du cercle ou carré)
auto_mode = False
t_auto = 0
auto_shape = None
angRi = np.array([0, 2*np.pi/3, 4*np.pi/3])
angAiEi = np.array([-np.pi/2, np.pi/6, 5*np.pi/6])

def rot2(a):   # fonction de roatation en 2D
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])

def inverse_kinematics(xE, yE, thE):      #Modele géometrique indirect 
    q = []
    RotEff = rot2(thE)              #rotation de l'effecteur 
    Transl = np.array([[xE], [yE]])             #translation de l'effeceur 
    THEff = np.block([[RotEff, Transl], [np.zeros((1, 2)), np.array([[1]])]])   # matrice homogéne de transformation de l'effecteur 
    for i in range(3):
        rot = rot2(angRi[i])
        pos = np.array([[Rb * np.cos(angAiEi[i])], [Rb * np.sin(angAiEi[i])]])
        TH = np.block([[rot, pos], [np.zeros((1, 2)), np.array([[1]])]])            #matrice homogene de transformation du bras i 
        PEi_E = np.array([[Re * np.cos(angAiEi[i])], [Re * np.sin(angAiEi[i])], [1]])
        PEi_0 = THEff @ PEi_E                               # Passage au repère monde
        PEi_i = np.linalg.inv(TH) @ PEi_0                   # Passage au repère du bras i 
        x, y = PEi_i[0, 0], PEi_i[1, 0]
        aux = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)    # on calcul beta en utilisant la loi des cosinus 
        if abs(aux) <= 1:
            beta = elbow * np.arccos(aux)
        elif abs(aux) > 1:
            return None                         #position innateignable 
        else:
            beta = 0
        alpha = np.arctan2(y, x) - np.arctan2(L2 * np.sin(beta), L1 + L2 * np.cos(beta))
        q.extend([alpha, beta])
    return q

def is_near_singularity(q):   #fonction evalue si la configuration actuelle est proche d'une singularité 
    threshold_parallel = 1e-3    #seuil pour detection singularité parrallele 
    threshold_series = 1e-2         # seuil pour detection singularité en série 
    A_matrix = []
    B_diag = []
    base_points = [np.array([0, -Rb]),                          # coordonnées des bases fixes 
                   np.array([Rb * np.sqrt(3)/2, Rb / 2]),
                   np.array([-Rb * np.sqrt(3)/2, Rb / 2])]
    for i in range(3):
        alpha, beta = q[2*i], q[2*i+1]
        A = base_points[i]
        R = rot2(angRi[i])                  # matrice de rotation pour le bras i
        u = R @ np.array([L1 * np.cos(alpha), L1 * np.sin(alpha)])
        B = A + u                               
        vect_AB = B - A                         #calcul du point B( fin du premier segment L1)
        gamma = np.arctan2(vect_AB[1], vect_AB[0])          #direction du bras dans le plan 
        norm_u = np.linalg.norm(vect_AB)
        d = -np.cross(A, vect_AB) / norm_u              # d : distance orientée entre l'origine et le vecteur AB (produit vectoriel 2D)

        A_matrix.append([np.cos(gamma), np.sin(gamma), d])
        ei = L1 * np.sin(beta)
        B_diag.append(ei)
    A_matrix = np.array(A_matrix)
    B_matrix = np.diag(B_diag)
    det_A = np.linalg.det(A_matrix)
    det_B = np.linalg.det(B_matrix)

     # Affichage des alertes si on détecte une singularité

    if abs(det_A) < threshold_parallel:
        print(f"\u26a0\ufe0f Singularité PARALLÈLE détectée (det(A) ≈ {det_A:.3e})")
        return True
    if abs(det_B) < threshold_series:
        print(f"\u26a0\ufe0f Singularité SÉRIE détectée (det(B) ≈ {det_B:.3e})")
        return True
    return False

def verify_geom(q, xE, yE, thetaE):     #verifie la validité géometrique de la configuartion actuelle( singularité ou collision)
    RotEff = rot2(thetaE)
    Transl = np.array([[xE], [yE]])
    THEff = np.block([[RotEff, Transl], [np.zeros((1, 2)), np.array([[1]])]])
    for i in range(3):
        alpha = q[2*i]
        beta = q[2*i + 1]
        rot = rot2(angRi[i])
        pos = np.array([[Rb * np.cos(angAiEi[i])], [Rb * np.sin(angAiEi[i])]])
        THRi_0 = np.block([[rot, pos], [np.zeros((1, 2)), np.array([[1]])]])

        # Calcul du point Ei dans le repère monde
        PEi_E = np.array([[Re * np.cos(angAiEi[i])], [Re * np.sin(angAiEi[i])], [1]])
        PEi_0 = THEff @ PEi_E #produit matriciel 

        # Calcul du point Bi (fin du 2e segment)
        PBi_i = np.array([[L1 * np.cos(alpha) + L2 * np.cos(alpha + beta)],
                          [L1 * np.sin(alpha) + L2 * np.sin(alpha + beta)], [1]])
        PBi_0 = THRi_0 @ PBi_i

        

        if np.linalg.norm(PEi_0[:2] - PBi_0[:2]) > 1e-2:  # Vérifie que Bi ≈ Ei (à epsilon près)
            return False
    return True             # Tous les bras respectent la contrainte géométrique

def detect_collisions(q):               # Detecte les collisions entre les bras 
    def segments_intersect(p1, p2, q1, q2): #Fonction qui vérifie si les deux segments [p1,p2] et [q1,q2] se croisent dans le plan.

        def ccw(a, b, c):  # vérifie si les points sont orientés dans le sens anti-horaire. ( counter clockwise)
            return (c[1]-a[1]) * (b[0]-a[0]) > (b[1]-a[1]) * (c[0]-a[0])

        # Deux segments s'intersectent si et seulement si les extrémités de chacun sont de part et d'autre de l'autre segment.
        return ccw(p1, q1, q2) != ccw(p2, q1, q2) and ccw(p1, p2, q1) != ccw(p1, p2, q2)


    def get_segment_points(alpha, beta, base_pos, rot):   # Calcule les 3 points clés d'un bras : base, articulation (coude), extrémité.

        joint1 = base_pos
        joint2 = base_pos + rot @ np.array([L1 * np.cos(alpha), L1 * np.sin(alpha)])
        joint3 = joint2 + rot @ np.array([L2 * np.cos(alpha + beta), L2 * np.sin(alpha + beta)])
        return [joint1, joint2, joint3]

    Rot1 = rot2(2*np.pi/3)
    Rot2 = rot2(4*np.pi/3)
    arms = [
        get_segment_points(q[0], q[1], np.array([0, -Rb]), np.eye(2)),
        get_segment_points(q[2], q[3], np.array([Rb*np.sqrt(3)/2, Rb/2]), Rot1),
        get_segment_points(q[4], q[5], np.array([-Rb*np.sqrt(3)/2, Rb/2]), Rot2)
    ]
    for i in range(3):
        for j in range(i+1, 3):
            for a in range(2):
                for b in range(2):
                    if segments_intersect(arms[i][a], arms[i][a+1], arms[j][b], arms[j][b+1]):
                        print(f"\u26a0\ufe0f Collision détectée entre bras {i+1} et {j+1}")
                        return True
    return False

def trace_rob(q, thetaE=0, danger=False):
    screen.fill(WHITE)    ## Efface l’écran à chaque frame pour redessiner depuis zéro
    alpha1, beta1, alpha2, beta2, alpha3, beta3 = q
    Rot1 = rot2(2 * np.pi / 3)
    Rot2 = rot2(4 * np.pi / 3)
    P10 = np.array([0, -Rb])
    P11 = P10 + np.array([L1 * np.cos(alpha1), L1 * np.sin(alpha1)])
    P12 = P11 + np.array([L2 * np.cos(alpha1 + beta1), L2 * np.sin(alpha1 + beta1)])
    P20 = np.array([Rb * np.sqrt(3)/2, Rb/2])
    P21 = P20 + Rot1 @ np.array([L1 * np.cos(alpha2), L1 * np.sin(alpha2)])
    P22 = P21 + Rot1 @ np.array([L2 * np.cos(alpha2 + beta2), L2 * np.sin(alpha2 + beta2)])
    P30 = np.array([-Rb * np.sqrt(3)/2, Rb/2])
    P31 = P30 + Rot2 @ np.array([L1 * np.cos(alpha3), L1 * np.sin(alpha3)])
    P32 = P31 + Rot2 @ np.array([L2 * np.cos(alpha3 + beta3), L2 * np.sin(alpha3 + beta3)])
    offset = np.array([400, 400])
    pygame.draw.line(screen, RED, P10 + offset, P11 + offset, 3)
    pygame.draw.line(screen, PINK, P11 + offset, P12 + offset, 2)
    pygame.draw.line(screen, RED, P20 + offset, P21 + offset, 3)
    pygame.draw.line(screen, PINK, P21 + offset, P22 + offset, 2)
    pygame.draw.line(screen, RED, P30 + offset, P31 + offset, 3)
    pygame.draw.line(screen, PINK, P31 + offset, P32 + offset, 2)
    color = YELLOW if danger else GREEN
    pygame.draw.polygon(screen, color, [P12 + offset, P22 + offset, P32 + offset])
    for p in trajectory:
        pygame.draw.circle(screen, BLUE, (int(p[0]+400), int(p[1]+400)), 1)
    screen.blit(font.render("Appuyez sur A pour dessiner un cercle", True, (0, 0, 0)), (20, 20))
    screen.blit(font.render("Appuyez sur B pour dessiner un carré", True, (0, 0, 0)), (20, 50))
    screen.blit(font.render("Appuyez sur Q pour tourner à gauche", True, (0, 0, 0)), (20, 110))
    screen.blit(font.render("Appuyez sur E pour tourner à droite", True, (0, 0, 0)), (20, 140))
    mode_text = f"Mode automatique : {auto_shape}" if auto_mode else "Mode manuel"

    screen.blit(font.render(mode_text, True, (0, 0, 0)), (20, 80))

    pygame.display.flip()

def det_A(g1, g2, g3, d1, d2, d3):    #calul determinant de la matrice A

    A = np.zeros((3, 3))
    A[0] = [np.cos(g1), np.sin(g1), d1]
    A[1] = [np.cos(g2), np.sin(g2), d2]
    A[2] = [np.cos(g3), np.sin(g3), d3]

    return np.linalg.det(A)


def det_B(e1, e2, e3):                      #Calcul determinant de la matrice B

    B = np.zeros((3,3))
    B[0,0] = e1
    B[1,1] = e2
    B[2,2] = e3

    return np.linalg.det(B)

def find_safe_theta(xE, yE, thetaE, step=0.05, max_attempts=20):

    for i in range(-max_attempts, max_attempts + 1):  #on teste plusieurs orientations proches
        test_theta = thetaE + i * step          # orientation candidate
        q_test = inverse_kinematics(xE, yE, test_theta)   # calcul de la configuration articulaire pour cette orientation

        #Si une configuration existe et qu'elle évite les singularités ET les collisions :
        if q_test and not is_near_singularity(q_test) and not detect_collisions(q_test):
            return test_theta, q_test
    return None, None      # Aucune orientation valide trouvée dans la plage de recherche


def optimize_orientation(prev_q, pos_eff, num_candidates=30):  #Recherche l'orientation optimale de l’effecteur (thetaE) pour éviter les singularités ou collisions.
    xE, yE, thetaE = pos_eff
    best_theta = thetaE                 #meilleur angle trouvé 
    best_q = None                       #meilleur config articulaire 
    min_joint_jump = float('inf')       # pour minimiser les changements brusques ( pour que ce soit plus lisse)
    detA_threshold = 0.01               # Seuil indicatif pour éviter les singularités parallèles

    for dtheta in np.linspace(-np.pi, np.pi, num_candidates): # Génère des angles candidats autour de l'orientation actuelle
        test_theta = thetaE + dtheta
        q_candidate = inverse_kinematics(xE, yE, test_theta)

        if not q_candidate:             # Si la cinématique inverse échoue, on passe
            continue
        if detect_collisions(q_candidate) or is_near_singularity(q_candidate):     # on verifie que cette config ne pose pas de problème
            continue
        if not verify_geom(q_candidate, xE, yE, test_theta):      # Vérifie que la géométrie est bien respectée (bras atteignent bien les bons points) 
            continue

        if prev_q is not None:          # Si une config précédente existe, on mesure à quel point cette candidate est différente
            joint_jump = np.linalg.norm(np.array(q_candidate) - np.array(prev_q))
        else:
            joint_jump = 0

        # On conserve la configuration qui minimise les variations d'angle (plus fluide)

        if joint_jump < min_joint_jump:
            min_joint_jump = joint_jump
            best_theta = test_theta
            best_q = q_candidate

    if best_q: 
        return best_theta, best_q      # Retourne la meilleure orientation trouvée
    else:
        return None, None

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                auto_mode = not auto_mode
                auto_shape = "circle" if auto_mode else None
            elif event.key == pygame.K_b:
                auto_mode = not auto_mode
                auto_shape = "square" if auto_mode else None

    keys = pygame.key.get_pressed()

    #mode automatique ( dessin de trajectoire défini ( cercle ou carré))
    if auto_mode:
        t_auto += 1
        if auto_shape == "circle":
            R = 50
            omega = 0.02
            new_xE = R * math.cos(omega * t_auto)
            new_yE = R * math.sin(omega * t_auto)
        elif auto_shape == "square":
            side = 100
            period = 200
            phase = (t_auto // period) % 4
            progress = (t_auto % period) / period
            if phase == 0:
                new_xE = -side/2 + progress * side
                new_yE = -side/2
            elif phase == 1:
                new_xE = side/2
                new_yE = -side/2 + progress * side
            elif phase == 2:
                new_xE = side/2 - progress * side
                new_yE = side/2
            else:
                new_xE = -side/2
                new_yE = side/2 - progress * side
        new_thetaE = 0
    else:   # mode manuel on dessine via le controle du clavier 
        dx, dy, dth = 0, 0, 0
        if keys[pygame.K_LEFT]: dx = -1
        if keys[pygame.K_RIGHT]: dx = 1
        if keys[pygame.K_UP]: dy = -1
        if keys[pygame.K_DOWN]: dy = 1
        if keys[pygame.K_q]: dth = -0.02
        if keys[pygame.K_e]: dth = 0.02

        # Mise à jour de la position et de l’orientation de l’effecteur
        new_xE = xE + dx
        new_yE = yE + dy
        new_thetaE = thetaE + dth

    q = inverse_kinematics(new_xE, new_yE, new_thetaE)
    if q and not is_near_singularity(q) and not detect_collisions(q) and verify_geom(q, new_xE, new_yE, new_thetaE):
        prev_q = q
        xE, yE, thetaE = new_xE, new_yE, new_thetaE
        trajectory.append(np.array([xE, yE]))
        trace_rob(q, thetaE, danger=False)
    else:
        safe_theta, q_safe = optimize_orientation(prev_q, [new_xE, new_yE, new_thetaE])
        if q_safe and not detect_collisions(q_safe) and verify_geom(q_safe, new_xE, new_yE, safe_theta):
            prev_q = q_safe
            xE, yE, thetaE = new_xE, new_yE, safe_theta
            trajectory.append(np.array([xE, yE]))
            trace_rob(q_safe, safe_theta, danger=False)
        else:
            print("Impossible d'éviter la singularité ou collision.")
            if q:
                trace_rob(q,thetaE, danger=True)

    clock.tick(60)

pygame.quit()
sys.exit()


