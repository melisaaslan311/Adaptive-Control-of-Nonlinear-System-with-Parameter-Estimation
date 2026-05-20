# Adaptive-Control-of-Nonlinear-System-with-Parameter-Estimation

---

## 📌 Overview

This repository implements a **Model Reference Adaptive Control (MRAC)** system in MATLAB/Simulink. The plant is a first-order nonlinear system with unknown parameters, and the controller adapts online to drive the tracking error to zero.

The system tracks a sinusoidal reference trajectory using a Lyapunov-based parameter update law.

---

## 🧮 System Description

### Plant (Nonlinear Model)

```
ẋ = f(x, t) + u
f(x, t) = a·x·cos(t) + b·sin(t)
```

| Parameter | True Value |
|-----------|-----------|
| `a_true`  | 2.0       |
| `b_true`  | 1.0       |

The parameters `a` and `b` are **unknown** to the controller. The adaptive law estimates them online as `â` (a_hat) and `b̂` (b_hat).

---

### Reference Trajectory

```matlab
xd   = sin(t)
dxd  = cos(t)
```

---

### Controller

The control law is:

```
u = ẋd − f̂(x, t) + k·e
```

where:
- `e = xd − x` is the tracking error  
- `f̂(x, t) = â·x·cos(t) + b̂·sin(t)` is the estimated nonlinearity  
- `k = 15` is the feedback gain

---

### Adaptive Parameter Update Law

Lyapunov-based gradient descent update rules:

```
â˙ = −γₐ · φₐ · e,    φₐ = x·cos(t)
b̂˙ = −γ_b · φ_b · e,  φ_b = sin(t)
```

| Gain      | Value |
|-----------|-------|
| `γₐ`      | 2     |
| `γ_b`     | 2     |

---

## 📁 File Structure

```
├── trajectory.m        % Reference trajectory generator (xd, dxd)
├── controller.m        % Control law computation (u, e)
├── parameter_update.m        % Parameter update law (dâ, db̂)
├── model.m         % Plant dynamics (ẋ = f(x,t) + u)
└── README.md
```

---

## 📊 Scoped Signals

The following signals are monitored via Simulink **Scope** blocks:

| Signal   | Description                        |
|----------|------------------------------------|
| `â`      | Estimated parameter (a_hat)        |
| `b̂`      | Estimated parameter (b_hat)        |
| `e`      | Tracking error `xd − x`            |

---

## 🔬 Expected Behavior

- **Tracking error `e`** converges to zero as the adaptive law refines the parameter estimates.
- **`â`** converges toward `2.0` (true value of `a`).
- **`b̂`** converges toward `1.0` (true value of `b`).
- The system is stable by construction via Lyapunov analysis.
<img width="1600" height="699" alt="image" src="https://github.com/user-attachments/assets/9f297580-5a31-4183-bbbf-74d1f69061dd" />
<img width="1600" height="825" alt="image" src="https://github.com/user-attachments/assets/2ef10e39-781b-4f86-925c-f47ffd2b798b" />
<img width="1600" height="849" alt="image" src="https://github.com/user-attachments/assets/1b11bbf3-71b3-4514-8b47-726605d56726" />
<img width="1600" height="847" alt="image" src="https://github.com/user-attachments/assets/6cfee9b4-0f14-435e-b26c-f44accfd7bba" />

---

## 🚀 How to Run

1. Open MATLAB and navigate to the project folder.
2. Open the Simulink model (`.slx` file).
3. Set simulation time (e.g., `T = 200`).
4. Run the simulation.
5. Observe `â`, `b̂`, and `e` in the Scope blocks.

---

## 📚 Theory

This controller is based on the **direct MRAC** methodology:

- A reference model defines the desired closed-loop behavior.
- The control law cancels estimated nonlinearities and adds proportional error feedback.
- The update law is derived from a **Lyapunov function** to guarantee stability:

```
V = (1/2)e² + (1/2γₐ)ãₐ² + (1/2γ_b)ã_b²
```

where `ã = a_true − â` are the parameter estimation errors. The derivative `V̇ ≤ 0` ensures closed-loop stability.

---

