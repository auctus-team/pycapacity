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
    equal-contrib: true
    affiliation: "1" # (Multiple affiliations must be quoted)
  - name: Vincent Padois
    equal-contrib: true # (This is how you can denote equal contributions between multiple authors)
    affiliation: 1
  - name: David Daney
    corresponding: true # (This is how to denote the corresponding author)
    affiliation: 1
affiliations:
 - name: INRIA Sud Ouest, Bordeaux, France
date: 02 May 2023
bibliography: paper.bib
---

# Summary

# Statement of need

# Implemented algorithms

## Hyper-plane shifting method

This is an algorihtm based on the paper by [@Gouttefarde2010] which presents an efficient way of determining the minimal half-space $\mathcal{H}$ representation of the polytope described by the equation 

\begin{equation}\label{eq:hpsm}
P = \{ x ~|~ x = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}

## Vertex enumeration auctus

This is an algorithm based on the paper by [@Skuric2021] which describes an efficient method for finding vertex $\mathcal{V}$ representation of the polytope described by the equation

\begin{equation}\label{eq:vertex_auctus}
P = \{ x ~|~ Ax = y, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}


## Iterative convex-hull method

This is an algorihtm descirbed in the paper by [@Skuric2022] which implements an efficient method which iteratively approximates the polytope

\begin{equation}\label{eq:ichm}
P = \{ x ~|~ Ax = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}

The method finds both vertex $\mathcal{V}$ and half-plane $\mathcal{H}$ representation of the polytope at the same time. 
  
And it can be additionally extended to the case where there is an additional projection matrix $P$ making a class of problems:

\begin{equation}\label{eq:ichm_full}
P = \{ x ~|~ x= Pz, Az = By, \quad y_{min}\leq y \leq y_{max} \}
\end{equation}





# Physical capacity metrics available


## Robotic manipulators metrics

For the robotic manipulators the package integrates several velocity, force and acceleration capacity calculation functions based on ellipsoids:

### Ellipsoids
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

### Polytopes
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
P_{\cap} = \mathcal{P}_{f1} \cap \mathcal{P}_{f1} \qquad P_{\oplus} = \mathcal{P}_{f1} \oplus \mathcal{P}_{f1} 
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

This approaxh is described in the paper by [Skuric2023].


Where $J$ is the robot jacobian matrix, $f$ is the vector of cartesian forces, $\dot{x}$ and $\ddot{x}$ are vectors fo cartesian velocities and accretions, $\dot{q}$ is the vector of the joint velocities and $\tau$ is the vector of joint torques.

## Human musculoskeletal model metrics

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

# Citations


If you want to cite a software repository URL (e.g. something on GitHub without a preferred
citation) then you can do it with the example BibTeX entry below for @fidgit.

For a quick reference, the following citation commands can be used:
- `@author:2001`  ->  "Author et al. (2001)"
- `[@author:2001]` -> "(Author et al., 2001)"
- `[@author1:2001; @author2:2001]` -> "(Author1 et al., 2001; Author2 et al., 2002)"

# Figures

Figures can be included like this:
![Caption for example figure.\label{fig:example}](figure.png)
and referenced from text using \autoref{fig:example}.

Figure sizes can be customized by adding an optional second parameter:
![Caption for example figure.](figure.png){ width=20% }

# Acknowledgements


# References