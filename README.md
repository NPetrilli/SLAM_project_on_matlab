# 🤖 SLAM Project — State Estimation and Filtering

This project tackles the classical **SLAM (Simultaneous Localization and Mapping)** problem in mobile robotics. A robot moves in an unknown 2D environment and must estimate both its own pose and a map of the environment using range measurements and odometry data. The work is divided into four challenges, progressively increasing in complexity and realism.

---

## 📚 Project Structure

The project follows four main challenges:

### Challenge A — Known Data Association
- Simulated environment with point landmarks
- Known data association (landmarks are labeled)
- Extended Kalman Filter (EKF) implemented for pose and landmark estimation
- Performance evaluated using ground truth and 3σ confidence intervals

### Challenge B — Unknown Data Association
- Simulated point landmarks without known IDs
- Dynamic state expansion as new landmarks are detected
- Data association performed using **Mahalanobis distance** with thresholds:
  - Association: τ₁ = 5.9915
  - New landmark insertion: τ₂ = 13.8155
- Correct identification reduces false landmarks compared to Challenge A

### Challenge C — Landmark Extraction from Simulated Lidar
- Lidar scans simulated in a realistic map of the San Niccolò building
- Corner detection via `islocalmin` function and custom filter for spurious walls
- EKF adapted with dynamic feature extraction and tuned confidence thresholds
- Comparison with real map and odometry-based mapping

### Challenge D — SLAM with Real-World Lidar Data
- Real data from a Pioneer LX robot in San Niccolò building
- Two real-world exercises:
  - **1-side corridor traversal**
  - **4-sides full floor navigation**
- Manual parameter tuning (Q, R, thresholds) required
- Loop closure evaluated qualitatively via trajectory correction and landmark consistency

---

## 🧠 Key Techniques

- **Motion model**: Unicycle with forward and angular velocities
- **Sensor model**: Lidar with noise in range and bearing
- **Estimation**: Extended Kalman Filter (EKF)
- **Data Association**: Mahalanobis distance + confidence thresholds
- **Landmark Extraction**: Local minima + geometric filtering
- **Visualization**: Trajectories, pose estimation, confidence ellipses

---

## 🛠️ Tools and Data

- **Language**: MATLAB
- **Data Files**:
  - `data_point_land_1.mat`, `data_point_land_2.mat` — Simulated with known associations
  - `data_sim_lidar_1.mat`, `data_sim_lidar_2.mat` — Lidar-based scans in simulated environment
  - `real_lidar_logs.txt` → parsed via `MainReadLogger.m`
- **Map Comparison**:
  - True map via `PlotMapSN(Obstacles)`
  - Estimated vs odometry-based scan reconstructions

---

## 📊 Results Summary

| Challenge | Dataset | Loop Closure | Accuracy | Observations |
|----------|---------|--------------|----------|--------------|
| A        | 1 & 2   | ❌            | ✅        | Some drift at turns, all landmarks detected |
| B        | 1 & 2   | ❌            | ✅        | Improved map with correct merging of duplicate landmarks |
| C        | 1 & 2   | ✅            | ✅        | Landmarks detected via corners, better filtering needed |
| D        | 1 & 2   | ✅            | ⚠️        | Odometry affects results, loop closure seen in 4-side map |

---

## 🔬 Lessons Learned

- High noise or inaccurate odometry affects estimation significantly
- Mahalanobis-based data association improves consistency
- Reliable landmark extraction is crucial for map quality
- Real data requires robust filtering and loop closure handling

---

## 👨‍💻 Authors

Niccolò Petrilli, Caciolli, Casini, Maccari  
*State Estimation and Filtering — University of Siena (2023–2024)*  
