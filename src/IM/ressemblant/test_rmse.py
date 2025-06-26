import argparse
import cv2
import numpy as np
import sys

def compute_rmse(img1: np.ndarray, img2: np.ndarray) -> float:
    """Compute the root mean square error between two images."""
    if img1.shape != img2.shape:
        raise ValueError("Les images doivent avoir la même taille pour calculer le RMSE.")
    diff = img1.astype(np.float32) - img2.astype(np.float32)
    mse = np.mean(np.square(diff))
    return float(np.sqrt(mse))

def merge_images(img1: np.ndarray, img2: np.ndarray) -> np.ndarray:
    """Merge two images with a 50/50 weighted average."""
    # Si tailles différentes, redimensionner img2 pour matcher img1
    if img1.shape != img2.shape:
        img2 = cv2.resize(img2, (img1.shape[1], img1.shape[0]), interpolation=cv2.INTER_NEAREST)
    merged = cv2.addWeighted(img1, 0.5, img2, 0.5, 0)
    return merged

def main():
    parser = argparse.ArgumentParser(
        description="Calcule le RMSE entre deux images PGM et merge si au-dessus d'un seuil."
    )
    parser.add_argument("img1", help="Chemin vers la première image (PGM).")
    parser.add_argument("img2", help="Chemin vers la seconde image (PGM).")
    parser.add_argument("threshold", type=float, help="Seuil de RMSE au-dessus duquel on merge.")
    parser.add_argument("output", help="Chemin de sortie pour l'image fusionnée (PGM).")
    args = parser.parse_args()

    # Chargement des images en niveaux de gris
    im1 = cv2.imread(args.img1, cv2.IMREAD_GRAYSCALE)
    if im1 is None:
        print(f"Erreur : impossible de charger {args.img1}", file=sys.stderr)
        sys.exit(1)

    im2 = cv2.imread(args.img2, cv2.IMREAD_GRAYSCALE)
    if im2 is None:
        print(f"Erreur : impossible de charger {args.img2}", file=sys.stderr)
        sys.exit(1)

    # Calcul du RMSE
    try:
        rmse_value = compute_rmse(im1, im2)
    except ValueError as e:
        print(f"Erreur : {e}", file=sys.stderr)
        sys.exit(1)

    print(f"RMSE entre {args.img1} et {args.img2} : {rmse_value:.2f}")

    # Décision de fusion
    if rmse_value > args.threshold:
        print(f"RMSE > seuil ({args.threshold}), fusion des images.")
        out_img = merge_images(im1, im2)
    else:
        print(f"RMSE ≤ seuil ({args.threshold}), aucune fusion. Sauvegarde de la première image.")
        out_img = im1.copy()

    # Sauvegarde de l'image de sortie
    success = cv2.imwrite(args.output, out_img)
    if not success:
        print(f"Erreur : impossible d'écrire {args.output}", file=sys.stderr)
        sys.exit(1)

    print(f"Image de sortie enregistrée sous {args.output}")

if __name__ == "__main__":
    main()

