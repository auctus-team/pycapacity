---
title: 'Pycapacity: a real-time task-space capacity calculation package for robotics and biomechanics'
tags:
  - Python
  - robotics
  - kinematics
  - polytope algebra
authors:
  - name: Antun Skuric
    corresponding: true # (This is how to denote the corresponding author)
    orcid: 0000-0002-3323-4482
    affiliation: 1 # (Multiple affiliations must be quoted)
  - name: Vincent Padois
    affiliation: 1
  - name: David Daney
    affiliation: 1
affiliations:
 - name: INRIA, Bordeaux, France
 - index: 1
date: 23 May 2023
bibliography: paper.bib
---

# Summary

There is a rising interest in collaborative robotics and physical human robot interaction, where the robot's are required to adapt to certain needs of the human in real-time. This adaptation raises a fundamental challenge: the ability to evaluate the need of assistance of the operator. One of ways to quantify the need of assistance is by evaluating the operator's physical abilities in real-time and comparing them to the required physical abilities required by the collaborative task. Then the robot can assist the operator where he lacks the physical ability to accomplish the task.

Beyond the characterization of human physical capabilities, as todays collaborative robotic manipulators are designed for safety, their performance characteristics are relatively limited with respect to the more standard industrial robots. Therefore it is becoming increasingly important to exploit their full (physical) abilities when executing the task.  

There are many different metrics available in the literature that might be used to characterize physical abilities: force capacity, velocity capacity, acceleration capacity, accuracy, stiffness etc. Most of these metrics can be represented by two families of geometric shapes ellipsoids [@yoshikawa1985manipulability] and polytopes [@chiacchio1997force]. It can be interesting to be able to compute these metrics off line for analysis purposes (workspace design, human motion and ergonomics analysis) as well as in interactive ways for control or user feedback. The dimensionality of the problem to solve  and the limited computation time, respectively for off-line and online applications, advocate for efficient tools to evaluate these metrics.

# Statement of need

This python package implements several different physical ability metrics based on ellipsoids and polytopes, for robotic manipulators and human musculoskeletal modes. All the algorithms are implemented in python, and having execution times of a fraction of the second, they are intended to be used in real-time applications such as robot control and visualization to the operator. The package can be easily interfaced with standard libraries for robotic manipulator rigid body simulation such as `robotic-toolbox` [@corke2021not] or `pinocchio` [@carpentier2019pinocchio], as well as human musculoskeletal model biomechanics softwares `opensim` [@delp2007opensim] and `biorbd` [@michaudBiorbd2021]. The package can also be used with the Robot Operating System (`ROS`) [@quigley2009ros].

The package additionally implements a set of visualization tools for polytopes and ellipsoids based on the python package `matplotlib` intended for fast prototyping and quick and interactive visualization.

This package has been used in several scientific papers, for real-time control of collaborative carrying using two Franka Emika Panda robots [@Skuric2021], for developing an assist-as-needed control strategy for collaborative carrying task of the human operator and the Franka robot [@Skuric2022]. The package has aslo been used to calculate the approximation of the robot's reachable space using convex polytope [@skuric2023].


# Implemented polytope evaluation algorithms

This package implements several algorithms for polytope evaluation 

- Hyper-Plane Shifting Method (HPSM)
- Vertex Enumeration Auctus (VEA)
- Iterative Convex Hull Method (ICHM)

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

The package implements different physical ability metrics for robotic manipulators and humans based on musculoskeletal models.

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


Where $J$ is the robot jacobian matrix, $f$ is the vector of cartesian forces, $\dot{x}$ and $\ddot{x}$ are vectors fo cartesian velocities and accelerations, $\dot{q}$ is the vector of the joint velocities and $\tau$ is the vector of joint torques.

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

The methods for resolution of the polytope based metrics depend on the family of problems they correspond to. In case of robotic manipulators the methods used are given in the following table.


Polytope Metric | Algorithm | Problem type | Execution time
--- | -- | ----- | ---
Velocity | HPSM | $x=By,~~ y_{min} \leq y \leq y_{max}$ | ~2ms
Acceleration |  HPSM | $x=By,~~ y_{min} \leq y \leq y_{max}$ | ~5ms
Force  | VEA | $Ax=b, ~~ b_{min} \leq b \leq y_{max}$| ~7ms
Force intersection |  VEA | $Ax=b,~~ b_{min} \leq b \leq b_{max}$ | ~80ms
Force sum |  VEA | $Ax=b,~~ b_{min} \leq b \leq b_{max}$ | ~15ms
Reachable space |  ICHM | $x=By,~~  y \in P_{y}$ | ~50ms

The average execution time is calculated using 7 dof Franka Emika panda robot, the model was used with `pinocchio` software. All the experiments are run on a computer equipped with 1.90GHz Intel i7-8650U processor.

In case of human musculoskeletal models the methods used are given in the table below.

Polytope Metric  | Algorithm | Problem type | Execution time 
-- | --- | ----- | ---
Force  | ICHM | $Ax=By,~~ y_{min} \leq y \leq y_{max}$ | ~ 200ms 
Acceleration |  HPSM or ICHM | $x=By,~~ y_{min} \leq y \leq y_{max}$ |  ~ 300ms 
Velocity | ICHM | $x=By,~~ y \in P_{y}$ | ~ 200ms

The average execution time was calculated using 50 muscle 7 dof musculoskeletal model introduced by [@holzbaur2005model], the model was used with `biorbd` biomechanics software. The experiments are run on a computer equipped with 1.90GHz Intel i7-8650U processor.

# Acknowledgements
This work has been funded by the BPI France Lichie project.

# References
