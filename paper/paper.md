---
title: 'Pycapacity: A real-time task-space capacity calculation package'
tags:
  - Python
  - robotics
  - kinematics
  - polytope algebra
authors:
  - name: Antun Skuric
    orcid: 0000-0000-0000-0000
    affiliation: 1 # (Multiple affiliations must be quoted)
  - name: Vincent Padois
    affiliation: 1
  - name: David Daney
    corresponding: true # (This is how to denote the corresponding author)
    affiliation: 1
affiliations:
 - name: INRIA Sud Ouest, Bordeaux, France
date: 15 May 2023
bibliography: paper.bib
---

ashjdsalhj 


# Summary

There is a rising interest in collaborative robotics and physical human robot interaction, where the robot's are required to adapt to certain needs of the human in real-time. The fundamental challenge being the ability to evaluate the need of assistance of the operator. One of ways to quantify the need of assistance is by evaluating the operator's physical abilities in real-time and comparing them to the required physical abilities required by the collaborative task. Then the robot can assist the operator where he lacks the physical ability to accomplish the task.

Furthermore, as todays collaborative robotic manipulators are designed for safety, their performance characteristics are relatively limited with respect to the more standard industrial robots. Therefore it is becoming increasingly important to exploit their full (physical) abilities when executing the task.  

There are many different physical ability metrics available in the literature that might be used in such way, such as: force capacity, velocity capacity, acceleration capacity, precision and similar. Most of these metrics can be represented by two families of geometric shapes ellipsoids [@yoshikawa1985manipulability] and polytopes [@chiacchio1997force]. However more efficient numerical tools are needed in order to evaluate these metrics in order to allow for more of their real-time applications, for example in robot control or interactive visualisaiton. Additionally efficient performance metrics can be further used for robot and workspace design as well as human motion and ergonomics analysis.

# Statement of need

Therefore this python package implements several different robotic manipulator and human musculoskeletal model based physical capacity metrics, based on ellipsoids and polytopes. All the algorithms are implemented in python, and having execution times of a fraction of the seconds,they are intended to be used in real-time applications such as robot control and visualization to the operator as well as in robot performance. The package can be easily interfaced with standard libraries for robotic manipulator rigid body simulation such as `robotic-toolbox` [@corke2021not] or `pinocchio` [@carpentier2019pinocchio], as well as human musculoskeletal model biomechanics software `opensim` [@delp2007opensim] and `biorbd` [@michaudBiorbd2021]. 

The package additionally implements a set of visualization tools for polytopes and ellipsoids based on the python package `matplotlib` intended for fast prototyping and quick and interactive visualization.

This package has been used in several scientific papers, for real-time control of collaborative carrying using two Franka Emika Panda robots [@Skuric2021], for developing an assist-as-needed control strategy for collaborative carrying task of the human operator and the Franka robot [@Skuric2022]. The package has been used to calculate the approximation of the robot's reachable space using convex polytope [@skuric2023].


# Implemented polytope evaluation algorithms

This package implements several different algorithms for polytope evaluation 

- Hyper-plane shifting method (HPSM)
- Vertex enumeration auctus (VEA)
- Iterative Convex Hull method (ICHM)

These algorithms are all implemented in python and used to evaluate different polytope based physical ability metrics. Additionally the algorithms are available to the users to be used standalone as well.

## Hyper-plane shifting method

This is an algorithm based on the paper by [@Gouttefarde2010] which presents an efficient way of determining the minimal half-space $\mathcal{H}$ representation of the polytope described by the equation 

\begin{equation}\label{eq:hpsm}
P = \{ x ~|~ x = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}

## Vertex enumeration auctus 

This is an algorithm based on the paper by [@Skuric2021] which describes an efficient method for finding vertex $\mathcal{V}$ representation of the polytope described by the equation

\begin{equation}\label{eq:vertex_auctus}
P = \{ x ~|~ Ax = y, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}


## Iterative convex-hull method

This is an algorithm described in the paper by [@Skuric2022] which implements an efficient method which iteratively approximates the polytope

\begin{equation}\label{eq:ichm}
P = \{ x ~|~ Ax = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}

The method finds both vertex $\mathcal{V}$ and half-plane $\mathcal{H}$ representation of the polytope at the same time. 
  
And it can be additionally extended to the case where there is an additional projection matrix $P$ making a class of problems:

\begin{equation}\label{eq:ichm_full}
P = \{ x ~|~ x= Pz, Az = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}





# Physical capacity metrics

The package several implements different physical ability metrics for robotic manipulators and humans based on musculoskeletal models.

## Robotic manipulators metrics

For the robotic manipulators the package integrates several velocity, force and acceleration capacity calculation functions based on ellipsoids.

![2D and 3D force polytopes and their ellipsoid counterparts for a 7 degrees of freedom (DoF) \textit{Franka Emika Panda} robot. Both polytopes and ellipsoids are calculated separately for the 3D and for each of the 2D reduced task-space cases. Both polytopes and ellipsoids take in consideration the true joint torque limits provided by the manufacturer. The underestimation of the true force capabilities of the robot by ellipsoids appears clearly.](robot.png){ width=50% }

### Ellipsoids

- Velocity (manipulability) ellipsoid  
\begin{equation}\label{eq:ev_r}
E_{v} = \{\dot{x} ~| \dot{x} = J\dot{q},~ ||\dot{q}||\leq1 \}
\end{equation}

- Acceleration (dynamic manipulability) ellipsoid  
\begin{equation}\label{eq:ea_r}
E_{a} = \{\ddot{x} ~| \ddot{x} = JM^{-1}\tau,~ ||\tau||\leq1 \}
\end{equation}

- Force ellipsoid 
\begin{equation}\label{eq:ef_r}
E_{f} = \{{f} ~| J^{T}f = \tau,~ ||\tau||\leq1 \}
\end{equation}

### Polytopes

- Velocity polytope  
\begin{equation}\label{eq:pv_r}
P_{v} = \{\dot{x} ~| \dot{x} = J\dot{q},~ \dot{q}_{min}\leq\dot{q}\leq\dot{q}_{max} \}
\end{equation}

- Acceleration polytope 

\begin{equation}\label{eq:pa_r}
P_{a} = \{\ddot{x} ~| \ddot{x} = JM^{-1}\tau,~ \tau_{min}\leq\tau\leq\tau_{max} \}
\end{equation}

- Force polytope  

\begin{equation}\label{eq:pf_r}
P_{f} = \{f ~| J^{T}f = \tau,~ \tau_{min}\leq\tau\leq\tau_{max} \}
\end{equation}

- Force polytopes *Minkowski sum and intersection*  

\begin{equation}\label{eq:psi_r}
P_{\cap} = {P}_{f1} \cap {P}_{f1} \qquad P_{\oplus} = {P}_{f1} \oplus {P}_{f1} 
\end{equation}

- Reachable space approximation of in the desired horizon of interest $\Delta t_{h}$ using the convex polytope formulation 

\begin{equation}\label{eq:prs_r}
\begin{split}
P_x = \{\Delta x~ |~ \Delta{x} &= JM^{-1}\tau \frac{\Delta t_{h}^2}{2},\\
  {\tau}_{min} &\leq \tau \leq {\tau}_{max},\\
   \dot{q}_{min} &\leq M^{-1}\tau \Delta t_{h}  \leq \dot{q}_{max}, \\
  {q}_{min} &\leq M^{-1}\tau \frac{\Delta t_{h}^2}{2}  \leq {q}_{max} \}\\
  \end{split}
\end{equation}

This approach is described in the paper by [@skuric2023].


Where $J$ is the robot jacobian matrix, $f$ is the vector of cartesian forces, $\dot{x}$ and $\ddot{x}$ are vectors fo cartesian velocities and accretions, $\dot{q}$ is the vector of the joint velocities and $\tau$ is the vector of joint torques.

## Human musculoskeletal model metrics

![Cartesian force polytope of a musculoskeletal model of both human upper limbs with 7Dof and 50 muscles each, visualized with `biorbd` The polytopes are scaled with a ratio 1m : 1000N.\label{fig:force_polytope_human}](bimanual1.png){ width=70% }

For the human musculoskeletal models this package implements the polytope metrics:

- Velocity polytope  
\begin{equation}\label{eq:pv_h}
P_{v} = \{\dot{x} ~|~\dot{l} = L\dot{q},~ \dot{x} = J\dot{q},~ \dot{q}_{min}, ~ \leq\dot{q}\leq\dot{q}_{max}, ~\dot{l}_{min}\leq\dot{l}\leq\dot{l}_{max} \}
\end{equation}

- Acceleration polytope   
\begin{equation}\label{eq:pa_h}
 P_{a} = \{\ddot{x} ~|~ \ddot{x} = JM^{-1}NF,~ F_{min}\leq F\leq F_{max} \}
\end{equation}

- Force polytope   
\begin{equation}\label{eq:pf_h}
P_{f} = \{f ~|~ J^Tf = NF,~ F_{min}\leq F\leq F_{max} \}
\end{equation}

Where $J$ is the model's jacobian matrix, $L$ si the muscle length jacobian matrix, $N= -L^T$ is the moment arm matrix, $f$ is the vector of cartesian forces, $\dot{x}$ and $\ddot{x}$ are vectors fo cartesian velocities and accretions, $\dot{q}$ is the vector of the joint velocities, $\tau$ is the vector of joint torques, $\dot{l}$ is the vector of the muscle stretching velocities and $F$ is the vector of muscular forces. 


## Polytope evaluation algorithms used


The methods for resolution of these polytopes are based on different algorithms.

In case of robotic manipulators the methods used are:

Robot Metric | Algorithm used | Problem type | Execution time ( 7dof robot )
--- | -- | ----- | ---
Velocity polytope | HPSM | $x=By,~~ y_{min} \leq y \leq y_{max}$ | 2ms
Acceleration polytope |  HPSM | $x=By,~~ y_{min} \leq y \leq y_{max}$ | 5ms
Reachable space polytope |  ICHM | $x=By,~~  y \in P_{y}$ | ~50ms
Force polytope  | VEA | $Ax=b, ~~ b_{min} \leq b \leq y_{max}$| ~7ms
Force polytope intersection |  VEA | $Ax=b,~~ b_{min} \leq b \leq b_{max}$ | ~80ms
Force polytope sum |  VEA | $Ax=b,~~ b_{min} \leq b \leq b_{max}$ | 15ms

The average execution time was calculated using 7 dof Franka Emika panda robot, the model was used with `pinocchio` software.

In case of human musculoskeletal models the methods used are:

Human Metric  | Algorithm used | Problem type | Execution time ( 50 muscle 7dof upper limb model )
--- | -- | ----- | ----
Force polytope  | ICHM | $Ax=By,~~ y_{min} \leq y \leq y_{max}$ | ~ 200ms (10N tolerance)
Acceleration polytope |  HPSM or ICHM | $x=By,~~ y_{min} \leq y \leq y_{max}$ |  ~ 300ms (HPSM) ~150ms (ICHM 50m/s^2 tolerance)
Velocity polytope | ICHM | $x=By,~~ y \in P_{y}$ | ~ 200ms (10m/s tolerance)

The average execution time was calculated using 50 muscle 7 dof musculoskeletal model introduced by [@holzbaur2005model], the model was used with `biorbd` biomechanics software.

# Acknowledgements
This work has been funded by the BPI France Lichie project.

# References
