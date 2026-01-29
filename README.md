# Robotics Perception: EKF & KF Ball Tracking Archive
This repository is a **personal archive** of specific modules I developed while contributing to the **ICHIRO ITS** (Humanoid Soccer Robot Team) codebase. 

> **Note:** These files are snippets from a larger, team repository.

## 🚀 Key Modules
In this archive, I implemented two approaches for ball tracking and one for landmark localization:

### 1. Ball Tracking (Basic Version) - `ekf_ball`
Despite the name, this module implements a more straightforward state estimation for the ball.
* **State Vector:** $[x, y, v_x, v_y]^T$.
* **Model:** Linear constant velocity.
* **Purpose:** Provides a stable and computationally efficient baseline for ball tracking.

### 2. Advanced Extended Kalman Filter - `ekf2_ball`
The "Real" EKF implementation designed to handle non-linearities in ball movement.
* **State Vector:** $[x, y, v, \theta]^T$ (Position, Velocity Magnitude, and Heading).
* **Non-Linearity:** Uses **Jacobian Matrices** ($F$) to linearize the motion model, especially for handling changes in heading ($\theta$).
* **Features:** Includes angle normalization and time-scaled process noise.

### 3. Landmark Localization - `ekf_landmark`
Estimates the robot's relative position to field landmarks.
* **Measurement Model:** Range and Bearing (Distance & Angle).
* **Logic:** Linearizes the measurement function ($H$) using partial derivatives (Jacobians) to map global coordinates to sensor readings.

## Tech Stack & Dependencies
* **Language:** C++11 or higher.
* **Library:** Relies on the internal `keisan` matrix library (ICHIRO ITS math utility).
* **Key Math:** Partial derivatives, Jacobian matrices, and Covariance tuning.

## How It Works
The filters follow the standard estimation cycle:
1. **Predict:** Project the state ahead based on the motion model.
2. **Update:** Correct the estimate using noisy sensor measurements.

## How To Tuning
  - R Matrix: Tuning the measurement noise is the priority (start with 0.08 or 0.01).
  - Q Matrix: Adjust the process noise (start with 1e-3) to balance between sensor trust and model prediction.
