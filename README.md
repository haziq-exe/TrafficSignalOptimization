# Traffic Signal Optimization via Simplex Method

<div align="center">
<img src="Diagram.png" alt="Visualization of traffic stop we wish to optimize" width="75%"></img>
<br>
<br>

[![Paper](https://img.shields.io/badge/Read_Paper-black?style=for-the-badge&logo=wordpress)](https://haziqmk.me/files/MTH382_Project.pdf)
![MATLAB](https://img.shields.io/badge/MATLAB-black?style=for-the-badge&logo=octave&logoColor=white)
![MTH382](https://img.shields.io/badge/∑_MTH382_Project-black?style=for-the-badge)

</div>

<!-- <a align="center" href="MTH382_Project.pdf" style=font-size=medium;">Link to Paper</a> -->


This repository contains the project report and MATLAB implementation for optimizing traffic signal timing at a two-phase intersection using **Linear Programming (LP)**. The core challenge involved linearizing the convex, non-linear Webster's Delay Formula using the Simplex method to find the optimal Green Time split and Cycle Length.

## Project Objective

The goal of this project was to minimize the total vehicle delay ($J$) at a four-way intersection subject to constraints on Cycle Length ($C$) and minimum Green Time ($g_{\text{min}}$).

$$
\text{Minimize } J = q_{NS} \cdot z_{NS} + q_{EW} \cdot z_{EW}
$$

Where:
* $q_{NS}$ and $q_{EW}$ are the fixed arrival flow rates (vehicles/hour).
* $z_{NS}$ and $z_{EW}$ are the average delay per vehicle (seconds).

The final optimal solution achieved a precision within **0.0062 seconds** of the true non-linear theoretical minimum.

---

## Methodology: Outer Linearization

Since the standard Webster delay formula is a **non-linear, convex function**, it cannot be optimized directly by the Simplex algorithm. We overcame this by employing **Outer Linearization** (also known as the Cutting Plane method).

### 1. Non-Linear Model (Webster's Delay)

The delay function for a single direction, $D(g, C)$, is based on the effective Green Time ($g$) and Cycle Length ($C$):

$$
D(g,C)=\frac{C(1-g/C)^{2}}{2(1-q/s)}
$$

### 2. Linear Approximation using Tangent Planes

We approximated the non-linear delay surface with a **polyhedral envelope** formed by numerous flat tangent planes.

For each of the thousands of sampled points $(g_i, C_i)$, a linear constraint was generated using the first-order Taylor expansion:

$$
z_{NS}\ge D(g_{i},C_{i})+\frac{\partial D}{\partial g_{NS}}(g_{NS}-g_{i})+\frac{\partial D}{\partial C}(C-C_{i})
$$

### 3. Simplex Formulation

The full problem was solved using the MATLAB `linprog` function (implementing the Simplex method), with the decision vector $X = [g_{NS}; g_{EW}; C; z_{NS}; z_{EW}]$.

**Constraints:**

1.  **Cycle Length Equality:** The Green Times must sum to the Cycle Length:
    $$g_{NS} + g_{EW} - C = 0$$
2.  **Delay Approximations:** A large set of linear inequalities ($A \cdot X \le b$) generated from the tangent planes, where the delay variables ($z$) are forced to be greater than or equal to the "height" of the tangent planes.
3.  **Bounds:** Minimum constraints on Green Time ($g_{\text{min}}=10s$) and Cycle Length ($C_{\text{min}}=40s$).

---

## Results and Verification

The optimal solution was found by generating **4,120 linear constraints** (40 samples for $C$ and 103 samples for $g$ per direction). Although as low 15 linear constraints can produce similar results.

### Optimal Signal Strategy

| Parameter | Simplex Result | Exact Non-Linear Optimum | Difference |
| :--- | :--- | :--- | :--- |
| **Cycle Length ($C$)** | 40.0000 s | 40.0000 s | 0.0000 s  |
| **Green NS ($g_{NS}$)** | 22.6010 s | 22.6072 s | 0.0062 s  |
| **Green EW ($g_{EW}$)** | 17.3990 s | 17.3928 s | 0.0062 s  |
| **Total Delay ($J$ / vehicle)** | 2.17 h/h | 2.17 h/h | - |

### Key Findings

* **Precision:** The linearization method successfully identified the global optimal strategy with extremely high precision.
* **Approximation Error:** The Simplex method's delay prediction often **underestimated** the actual Webster delay , which is an expected consequence of using tangent planes (lower bounds) to approximate a convex curve. As the number of constraints increased (e.g., from 15 to 4120), the accuracy rapidly improved.

---

## Repository Contents

* `MTH382_Project.pdf`: The full project report detailing the background, derivation, and results.
* `main_optimization_code.m`: MATLAB script containing the Simplex setup, the loop to generate thousands of linear constraints, and the call to `linprog`.
* `visualization_code.m`: MATLAB script used to generate the 3D plots demonstrating how the tangent planes approximate the non-linear delay surface.
  
---

## Running the Code

The main optimization requires the MATLAB Optimization Toolbox for `linprog`.

1.  Clone this repository.
2.  Run `main_optimization_code.m` to calculate the optimal signal strategy.
3.  Run `visualization_code.m` to generate the 3D figures showing the tangent plane approximation.

NOTE: In the appendix you find details as to how you can change number of constraints and approximations in each file

---
