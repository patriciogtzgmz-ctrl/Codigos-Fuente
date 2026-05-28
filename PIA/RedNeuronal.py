import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

# ─────────────────────────────────────────────
#  CARGAR DATOS
# ─────────────────────────────────────────────

df = pd.read_csv("datos.csv")

X = df[["H", "S", "V"]].values.astype(np.float32)
y = df["amarillo"].values.astype(np.float32)

# Normalización min-max
X_min = X.min(axis=0)
X_max = X.max(axis=0)
X_norm = (X - X_min) / (X_max - X_min + 1e-8)

# División train / validación  (80 / 20)
np.random.seed(42)
idx = np.random.permutation(len(X_norm))
corte = int(len(idx) * 0.8)
X_train, X_val = X_norm[idx[:corte]], X_norm[idx[corte:]]
y_train, y_val = y[idx[:corte]],     y[idx[corte:]]

print(f"Train: {len(X_train)} muestras  |  Val: {len(X_val)} muestras")
print(f"Amarillos en train: {int(y_train.sum())}  |  en val: {int(y_val.sum())}")

# ─────────────────────────────────────────────
#  PESO DE CLASE (desbalance 21/79)
# ─────────────────────────────────────────────

n_pos = int(y_train.sum())
n_neg = len(y_train) - n_pos
peso_pos = n_neg / n_pos
pesos_clase = {0: 1.0, 1: peso_pos}
print(f"Peso clase positiva: {peso_pos:.2f}\n")

# ─────────────────────────────────────────────
#  MODELO
# ─────────────────────────────────────────────

modelo = keras.Sequential([
    layers.Input(shape=(3,)),
    layers.Dense(16, activation="relu"),
    layers.Dense(8,  activation="relu"),
    layers.Dense(1,  activation="sigmoid"),
], name="HSV_Amarillo")

modelo.compile(
    optimizer=keras.optimizers.Adam(learning_rate=0.001),
    loss="binary_crossentropy",
    metrics=["accuracy",
             keras.metrics.Precision(name="precision"),
             keras.metrics.Recall(name="recall")],
)

modelo.summary()

# ─────────────────────────────────────────────
#  ENTRENAMIENTO
# ─────────────────────────────────────────────

historial = modelo.fit(
    X_train, y_train,
    validation_data=(X_val, y_val),
    epochs=200,
    batch_size=16,
    class_weight=pesos_clase,
    verbose=1,
)

# ─────────────────────────────────────────────
#  EVALUACIÓN FINAL
# ─────────────────────────────────────────────

print("\n── Evaluación en validación ──")
resultados = modelo.evaluate(X_val, y_val, verbose=0)
nombres = modelo.metrics_names
for n, v in zip(nombres, resultados):
    print("  {n}: {v:.4f}")

# Matriz de confusión manual
y_pred_prob = modelo.predict(X_val, verbose=0).flatten()
y_pred = (y_pred_prob >= 0.5).astype(int)
y_true = y_val.astype(int)

tp = int(np.sum((y_pred == 1) & (y_true == 1)))
tn = int(np.sum((y_pred == 0) & (y_true == 0)))
fp = int(np.sum((y_pred == 1) & (y_true == 0)))
fn = int(np.sum((y_pred == 0) & (y_true == 1)))

print("\n  Matriz de confusión:")
print("              Pred 0   Pred 1")
print("  Real 0   :   {tn:4d}     {fp:4d}")
print("  Real 1   :   {fn:4d}     {tp:4d}")

# ─────────────────────────────────────────────
#  GRÁFICAS
# ─────────────────────────────────────────────

h = historial.history
epocas = range(1, len(h["loss"]) + 1)

fig, axes = plt.subplots(1, 2, figsize=(12, 4))
fig.suptitle("Red Neuronal HSV → Amarillo", fontsize=14, fontweight="bold")

# — Pérdida —
axes[0].plot(epocas, h["loss"],     label="Train loss",  color="#3182CE")
axes[0].plot(epocas, h["val_loss"], label="Val loss",    color="#E53E3E", linestyle="--")
axes[0].set_title("Pérdida (BCE)")
axes[0].set_xlabel("Época")
axes[0].set_ylabel("Loss")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# — Accuracy —
axes[1].plot(epocas, h["accuracy"],     label="Train acc", color="#38A169")
axes[1].plot(epocas, h["val_accuracy"], label="Val acc",   color="#D69E2E", linestyle="--")
axes[1].set_title("Accuracy")
axes[1].set_xlabel("Época")
axes[1].set_ylabel("Accuracy")
axes[1].set_ylim(0, 1.05)
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("entrenamiento_hsv.png", dpi=150, bbox_inches="tight")
plt.show()
print("\nGráfica guardada: entrenamiento_hsv.png")

# ─────────────────────────────────────────────
#  GUARDAR MODELO
# ─────────────────────────────────────────────

modelo.save("modelo_hsv.keras")
np.save("hsv_normalizacion.npy", np.array([X_min, X_max]))
print("Modelo guardado: modelo_hsv.keras")

# ─────────────────────────────────────────────
#  PREDICCIÓN DE EJEMPLO
# ─────────────────────────────────────────────

print("\n── Ejemplos de predicción ──")
ejemplos = np.array([
    [120.0, 0.64, 0.61],
    [315.0, 0.74, 0.85],
    [62.86, 0.78, 0.42],
    [342.0, 0.77, 0.82],
], dtype=np.float32)

ejemplos_norm = (ejemplos - X_min) / (X_max - X_min + 1e-8)
probs = modelo.predict(ejemplos_norm, verbose=0).flatten()

for hsv, prob in zip(ejemplos, probs):
    clase = "AMARILLO ✓" if prob >= 0.5 else "No amarillo"
    print(f"  H={hsv[0]:6.2f} S={hsv[1]:.2f} V={hsv[2]:.2f}  →  {prob*100:5.1f}%  {clase}")
    
# ─────────────────────────────────────────────
#  PESOS DE LA RED
# ─────────────────────────────────────────────

for i, capa in enumerate(modelo.layers):
    pesos = capa.get_weights()
    if pesos:
        W, b = pesos
        print(f"\nCapa {i+1} — {capa.name}  ({W.shape})")
        print(f"  Pesos W:\n{W}")
        print(f"  Biases b:\n{b}")