# Robot 3RRR – Modélisation et Simulation

> Projet de modélisation d’un robot parallèle 3RRR avec visualisation en temps réel sous Python.

## 🛠️ Description

Ce projet vise à concevoir, modéliser et simuler un robot manipulateur parallèle plan de type **3RRR**. L’objectif est de fournir un outil interactif capable de simuler le comportement du robot, en prenant en compte la cinématique, les singularités, les collisions, et l’optimisation de trajectoires.

Réalisé dans le cadre du cours de modélisation (2024/2025) par **Sayf Chafik**.

---

## 📷 Aperçu

Interface graphique en temps réel (mode manuel et automatique avec trajectoires prédéfinies) :

- Bras articulés et effecteur mobile.
- Détection de singularités et collisions.
- Affichage dynamique des trajectoires.

---

## ⚙️ Fonctionnalités

- **Modélisation 3D** sous SolidWorks optimisée pour l’impression 3D.
- **Simulation interactive** avec pygame :
  - Contrôle manuel via le clavier (flèches + Q/E pour rotation).
  - Suivi automatique de trajectoires (cercle ou carré).
- **Cinématique inverse** analytique.
- **Détection et évitement de singularités**.
- **Détection de collisions** (intersections des bras).
- **Optimisation automatique** de l’orientation de l’effecteur.

---

## Contrôles Clavier

| Touche | Action                          |
|--------|---------------------------------|
| ↑ ↓ ← → | Déplacement de l’effecteur       |
| Q / E  | Rotation horaire / anti-horaire |
| A      | Lancer une trajectoire circulaire |
| B      | Lancer une trajectoire carrée     |

---

##  Limitations

- Détection de collisions perfectible (pas de zones tampon).
- Pas d’affichage de l’espace de travail atteignable.
- Interface simple (pygame), sans sliders ni boutons GUI.

---

## Améliorations possibles

- Export CSV des trajectoires.
- Interface avancée avec `Tkinter` ou `PyQt`.
- Couplage avec robot réel via communication série.
- Visualisation des zones de singularité et de sécurité.

---

## Rapport

Le rapport complet du projet (modélisation, simulation, résultats, perspectives) est disponible dans `Projet_modelisation.pdf`.

---

## Auteur

- **Sayf Chafik** – Projet académique dans le cadre de l'Ue de Modelisation 2024/2025


