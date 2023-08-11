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
   index: 1
date: 23 May 2023
bibliography: paper.bib
---

# Summary

This paper presents a Python package called `pycapacity`, which provides a set of tools for evaluating task space physical ability metrics for humans and robots, based on polytopes and ellipsoids. The aim of `pycapacity` is to provide a set of efficient tools for their evaluation in an easy to use framework that can be easily integrated with standard robotics and biomechanics libraries. The package implements several state of the art algorithœ for polytope evaluation that bring many of the polytope metrics to the few milliseconds evaluation time, making it possible to use them in online and interactive applications. 

The package can be easily interfaced with standard libraries for robotic manipulator rigid body simulation such as `robotic-toolbox` [@corke2021not] or `pinocchio` [@carpentier2019pinocchio], as well as human musculoskeletal model biomechanics softwares `opensim` [@delp2007opensim] and `biorbd` [@michaudBiorbd2021]. The package can also be used with the Robot Operating System (`ROS`) [@quigley2009ros].

The package additionally implements a set of visualization tools for polytopes and ellipsoids based on the Python package `matplotlib` intended for fast prototyping and quick and interactive visualization.


# Statement of need

There is a rising interest in collaborative robotics and physical human robot interaction, where the robots are often required to adapt to certain needs of the human in real-time. This adaptation raises a fundamental challenge: the ability to evaluate the need of assistance of the operator. One way to quantify the need of assistance is by evaluating the operator's physical abilities in real-time and comparing them to the physical abilities required to execute the task. Having this real-time information enables creating collaborative robot control strategies that assist the operators by compensating for their lacking physical ability to accomplish the tasks.

Beyond the characterization of human physical capabilities, as today's collaborative robotic manipulators are designed for safety, their performance characteristics are relatively limited with respect to the more standard industrial robots. Therefore it is becoming increasingly important to exploit their full (physical) abilities when executing the task.  

There are many different metrics available in the literature that might be used to characterize physical abilities: force capacity, velocity capacity, acceleration capacity, accuracy, stiffness etc. Most of these metrics can be represented by two families of geometric shapes: ellipsoids [@yoshikawa1985manipulability] and polytopes [@chiacchio1997force]. These metrics are traditionally important tools for off-line analysis purposes (workspace design, human motion and ergonomics analysis) and recently, they have shown a great potential to be used for interactive online applications, to be integrated in robot control strategies or as a visual feedback to the operator. 

Ellipsoid metrics are often used for evaluating the manipulability of the robot's end-effector. The manipulability ellipsoid is a geometric shape that represents the robot's ability to move with in the task space. The manipulability ellipsoid is defined by its principal axis that can be found very efficiently using the singular value decomposition (SVD) of the robot's Jacobian matrix [@yoshikawa1985manipulability]. Due to their computational efficiency and intuitive visualisation, they have been used in many different applications, such as robot control, workspace design, robot design, etc. Therefore there are several open-source packages that implement the manipulability ellipsoid evaluation and visualisation, such as `MMC` [@Haviland2020Maximising], `manipulability_metrics` [@manipulability_metrics], `Manipulability` [@manipulability;@Jaquier2021Geometry].However, all of these packages are limited to the evaluation of the manipulability ellipsoid, representing the velocity capacity, and they do not provide tools for evaluating other ellipsoid metrics, such as force capacity, acceleration capacity, etc. Additionally these software packages are often developed for the use with a specific robotics library, such as `robotic-toolbox` [@corke2021not] or `ROS` [@carpentier2019pinocchio], and they are not trivial to integrate with other libraries.

Even though different efficient tools for evaluating ellipsoids are widely available in the literature and open-source community, the tools for evaluating polytopes are still relatively scarce. The main reason for this is that the polytopes are in general more complex to evaluate and manipulate than ellipsoids. However, the polytopes are much more accurate representation of the true limits. Additionally, polytopes are easy to visualize, as they are essentially a triangulated meshes, and they can be easily integrated in the robot control strategies, as they can be expressed as a set of linear constraints. 

The evaluation of polytopes is often a computationally expensive task, as their resolution require using different vertex and facet enumeration algorithms [@fukuda2004frequently]. Therefore, their computation time is often the limiting factor for the use of polytopes in real world applications, especially when it comes to their online use. Furthermore, even though there are several open-source projects that implement polytope evaluation algorithms, such as `pypoman` [@pypoman], Multi-Parametric Toolbox 3 [@mpt3] or `cddlib`[@cddlib;@fukuda1997cdd], they are often very generic and not easy to use with standard physical ability polytopes. On the other hand, more specific polytope resolution software solutions, such as Constrained Manipulability package [@Long2018Evaluating; @constrained_manipulability] or `pygradientpolytope`[@pygradientpolytope], are often very specific to their applications, they lack the documentation and they are not easy to integrate with other libraries.

Therefore, this paper presents a Python `pycapacity` package in an effort to provide a set of tools specifically tailored for evaluating task space physical ability metrics for humans and robots, based on polytopes and ellipsoids. This package groups a set of efficient algorithms for their evaluation in an easy to use framework that can be easily integrated with standard robotics and biomechanics libraries. Futhermore, the package implements several state of the art algorithms for polytope evaluation that bring many of the polytope metrics to the few milliseconds evaluation time, making it possible to use them in online and interactive applications. 

`pycapacity` has been used in several scientific papers, for real-time control of collaborative carrying using two Franka Emika Panda robots [@Skuric2021], for developing an assist-as-needed control strategy for collaborative carrying task of the human operator and the Franka robot [@Skuric2022]. The package has also been used to calculate the approximation of the robot's reachable space using convex polytope [@skuric2023]. On the other hand, the package has been used for the biomechanical calibration of the human musculoskeletal model [@laisne2023genetic].

# Ellipsoids and polytopes as physical ability metrics

In robotics, different task space physical ability metrics establish the relationship between different limits of robot's actuators (joint positions, velocities, torques, etc.), its kinematics and dynamics, and the achievable sets of different task related physical quantities, such as achievable positions, velocities, forces and similar. Similar metrics can be established for humans as well, by leveraging their musculoskeletal models. Where the humans in addition to the joint limits (joint positions and velocities) have additional limits due to their using their muscles as actuators (contraction forces and velocities).

When it comes to characterizing these achievable sets, the two most common approaches are using ellipsoids and polytopes. Ellipsoids are often used to represent the robot's velocity capacity, so called manipulability, while polytopes are used to represent the robot's force capacity. However, both ellipsoids and polytopes can be used to represent any of the task space physical ability.

![An example manipulability polytope and ellipsoid geometry for a planar $m=2$ robot with $n=2$. The difference between the joint space limits for ellipsoid described with $||\dot{{q}}||_2\leq1$ (orange) and the range limits ${-1}\leq\dot{{q}}\leq{1}$ (blue) is shown on the right. The difference in obtained achievable task space velocity $\dot{{x}}$ polytope ${P}$ (blue) and ellipsoid ${E}$ (orange) is shown on the right plot. The plots show that both in joint and task space the ellipsoid metric is an underestimation of the true robot's capacity.](ellip_poly.png){ width=100% }

One of the most well known ellipsoid metrics is the manipulability ellipsoid [@yoshikawa1985manipulability], which is defined as the set of all achievable task space velocities ${\dot{x}}$ for a given robot configuration ${q}$ and joint velocity limits ${\dot{q}}$, and it can be expressed as:

\begin{equation}\label{eq:manip_ellipsoid}
E = \{ \dot{x} ~|~ \dot{x} = J({q})\dot{q}, \quad ||\dot{q}||_2 \leq 1 \}
\end{equation}

The equivalent polytope representation of the manipulability ellipsoid is the manipulability polytope, which is defined as the set of all achievable task space velocities ${\dot{x}}$ for a given robot configuration ${q}$ and joint velocity limits ${\dot{q}}$, and it can be expressed as:

\begin{equation}\label{eq:manip_polytope}
P = \{ \dot{x} ~|~ \dot{x} = J({q})\dot{q}, \quad -1 \leq \dot{q} \leq 1 \}
\end{equation}

Figure 1. illustrates the difference between the manipulability ellipsoid and polytope for a planar robot with two joints. The manipulability ellipsoid is an underestimation of the true robot's capacity, as it considers that the robot's velocity limits have the shape of a sphere, while in reality the robot's velocity limits are a cube. The manipulability polytope is a more accurate representation of the robot's capacity, as it considers the true shape of the robot's velocity limits. 

More generally, polytope based representations of different physical abilities present the exact solution both for robots and for human musculoskeletal models, while ellipsoids present an approximation. 
Figure 2. shows the difference between the force ellipsoid and polytope [@chiacchio1997force] for one configuration of the Franka Emika Panda robot.

Ellipsoids, however, are much more present in the literature, as their computation is much faster than the computation of polytopes. 

# Evaluating ellipsoids

Evaluating ellipsoids is a computationally efficient task, as it can be done using the singular value decomposition (SVD) [@yoshikawa1985manipulability]. Ellipsoids can be fully defined using its principal axis and principal axis lengths. Once they are known, the ellipsoid can be easily visualized and used for further analysis.

This package provides tools for evaluating several common ellipsoid metrics for robots and humans, such as velocity (manipulability), force and acceleration, and it provides a set of tools for their easy visualization. 

## Evaluating polytopes

Evaluating polytopes consists in finding either the minimal set of their vertices, $\mathcal{V}$-representation, or the minimal set of the half-planes defining their faces , $\mathcal{H}$-representation. The $\mathcal{V}$-representation is often used for visualization purposes, while the $\mathcal{H}$-representation is often integrated in different optimization problems, as it can be represented as a set of linear inequalities.

However, finding the $\mathcal{V}$-representation or the $\mathcal{H}$-representation of a polytope is a computationally expensive task, relying on different vertex and facet enumeration algorithms [@fukuda2004frequently]. The computational complexity of these algorithms depends on the polytope formulation, the dimensionality of the input (number of robot's joints or human muscles) and output spaces (1D, 2D, 3D or 6D Cartesian space) and the complexity of the polytope geometry (number of vertices and faces). 

Therefore, polytope evaluation is often a bottleneck in the computation of different physical ability metrics, especially for human musculoskeletal models, which have a large number of degrees of freedom and a large number of muscles. Furthermore, due to the inherent complexity of the polytope evaluation algorithms, finding the appropriate algorithm for a given polytope formulation and dimensionality of the input and output spaces is not a trivial task.

This package aims to provide a selection of algorithms for polytope evaluation, capable of evaluating common physical ability polytopes in an easy to use and efficient way. These algorithms are implemented in Python and can be used as standalone tools as well. Additionally, the package provides tools for easy visualization the 2D and 3D polytopes.

# Implemented physical capacity metrics

The package implements different physical ability metrics for robotic manipulators and humans based on musculoskeletal models.

## Robotic manipulators metrics

For robotic manipulators the package integrates several velocity, force and acceleration capacity calculation functions based on ellipsoids.

![2D and 3D force polytopes and their ellipsoid counterparts for a 7 degrees of freedom (DoF) \textit{Franka Emika Panda} robot. Both polytopes and ellipsoids are calculated separately for the 3D and for each of the 2D reduced task-space cases. Both polytopes and ellipsoids take in consideration the true joint torque limits provided by the manufacturer. The underestimation of the true force capabilities of the robot by ellipsoids appears clearly.](robot.png){ width=70% }

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

- Robot's reachable space approximation in the desired horizon of interest $\Delta t_{h}$ using the convex polytope formulation, described in the paper by [@skuric2023]

\begin{equation}\label{eq:prs_r}
\begin{split}
P_x = \{\Delta x~ |~ \Delta{x} &= JM^{-1}\tau \frac{\Delta t_{h}^2}{2},\\
  {\tau}_{min} &\leq \tau \leq {\tau}_{max},\\
   \dot{q}_{min} &\leq M^{-1}\tau \Delta t_{h}  \leq \dot{q}_{max}, \\
  {q}_{min} &\leq M^{-1}\tau \frac{\Delta t_{h}^2}{2}  \leq {q}_{max} \}\\
  \end{split}
\end{equation}

Where $J$ is the robot Jacobian matrix, $f$ is the vector of Cartesian forces, $\dot{x}$ and $\ddot{x}$ are vectors fo Cartesian velocities and accelerations, $\dot{q}$ is the vector of the joint velocities and $\tau$ is the vector of joint torques.

## Human musculoskeletal model metrics

![Cartesian force polytope of a musculoskeletal model of both human upper limbs with 7Dof and 50 muscles each, visualized with `biorbd`. The polytopes are scaled with a ratio 1m : 1000N.\label{fig:force_polytope_human}](bimanual1.png){ width=70% }

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

Where $J$ is the model's Jacobian matrix, $L$ si the muscle length Jacobian matrix, $N= -L^T$ is the moment arm matrix, $f$ is the vector of Cartesian forces, $\dot{x}$ and $\ddot{x}$ are vectors fo Cartesian velocities and accretions, $\dot{q}$ is the vector of the joint velocities, $\tau$ is the vector of joint torques, $\dot{l}$ is the vector of the muscle stretching velocities and $F$ is the vector of muscular forces. 

# Implemented polytope evaluation algorithms

This package implements several algorithms for polytope evaluation 

- Hyper-Plane Shifting Method (HPSM)
- Vertex Enumeration Algorithm (VEPOLI$^2$)
- Iterative Convex Hull Method (ICHM)

These algorithms are all implemented in Python and used to evaluate different polytope based physical ability metrics. Additionally, the algorithms are available to the users to be used standalone as well.

## Hyper-plane shifting method (HPSM)

This is an algorithm based on the paper by [@Gouttefarde2010] which presents an efficient way of determining the minimal half-space $\mathcal{H}$ representation of the polytope described by the equation 

\begin{equation}\label{eq:hpsm}
P = \{ x ~|~ x = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}

## Vertex enumeration algorithm (VEPOLI$^2$) 

This is an algorithm based on the paper by [@Skuric2021] which describes an efficient method for finding vertex $\mathcal{V}$ representation of the polytope described by the equation

\begin{equation}\label{eq:vertex_vepoli2}
P = \{ x ~|~ Ax = y, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}


## Iterative convex-hull method (ICHM)

This is an algorithm described in the paper by [@Skuric2022] which implements an efficient method which iteratively approximates the polytope

\begin{equation}\label{eq:ichm}
P = \{ x ~|~ Ax = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}

The method finds both vertex $\mathcal{V}$ and half-plane $\mathcal{H}$ representation of the polytope at the same time. 
  
It can be additionally extended to the case where there is an additional projection matrix $P$ making a class of problems:

\begin{equation}\label{eq:ichm_full}
P = \{ x ~|~ x= Pz, Az = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}



## Polytope evaluation algorithms used

The methods for resolution of the polytope based metrics depend on the family of problems they correspond to. In case of robotic manipulators the methods used are given in the following table.


Polytope Metric | Algorithm | Problem type | Execution time [ms] <br> mean $\pm$ std. (max)
--- | -- | ----- | ----
Velocity | HPSM | $x=By,~ y \in [y_{min}, y_{max}]$ | 3.6 $\pm$ 0.21 (5.7)
Acceleration |  HPSM | $x=By,~ y \in [y_{min}, y_{max}]$ | 6.6 $\pm$ 1.4 (14.2)
Force  | VEPOLI$^2$ | $Ax=b, ~ y \in [y_{min}, y_{max}]$| 6.8 $\pm$ 0.88 (16.4)
Force intersection |  VEPOLI$^2$ | $Ax=b,~ b \in [b_{min}, b_{max}]$ | 98.2 $\pm$ 29.33 (165.8)
Force sum |  VEPOLI$^2$ | $Ax=b,~ b \in [b_{min}, b_{max}]$ | 17.1 $\pm$ 3.4 (44.9)
Reachable space |  ICHM | $x=By,~  y \in P_{y}$ | 30.5 $\pm$ 6.6 (76.7)

The average execution time is calculated using 7 dof Franka Emika panda robot, the model was used with `pinocchio` software. All the experiments are run on a computer equipped with 1.90GHz Intel i7-8650U processor.

In case of human musculoskeletal models the methods used are given in the table below.

Polytope Metric  | Algorithm | Problem type | Execution time <br> mean $\pm$ std. (max) [ms]
-- | --- | ----- | ---
Force  | ICHM | $Ax=By,~ y \in [y_{min}, y_{max}]$ | 186.8 $\pm$ 45.6 (281.6)
Acceleration |  HPSM or ICHM | $x=By,~ y \in [y_{min}, y_{max}]$ |  378.8 $\pm$ 62.3 (643.7)
Velocity | ICHM | $x=By,~ y \in P_{y}$ | 223.1 $\pm$ 60.4 (389.1)

The average execution time was calculated using 50 muscle 7 dof musculoskeletal model introduced by [@holzbaur2005model], the model was used with `biorbd` biomechanics software. The experiments are run on a computer equipped with 1.90GHz Intel i7-8650U processor. 


# Conclusion


# Acknowledgements
This work has been funded by the BPI France Lichie project.

# References
