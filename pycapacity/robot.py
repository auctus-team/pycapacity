"""
Overview
---------

This is a pyhton module which implements different robot performance metrics based on polytopes and ellipsoids.

* acceleration `polytope <#pycapacity\.robot\.acceleration_polytope>`_ and `ellipsoid <#pycapacity\.robot\.acceleration_ellipsoid>`_
* velocity (manipulability) `polytope <#pycapacity\.robot\.velocity_polytope>`_ and `ellipsoid <#pycapacity\.robot\.velocity_ellipsoid>`_
* force `polytope <#pycapacity\.robot\.force_polytope>`_ and `ellipsoid <#pycapacity\.robot\.force_ellipsoid>`_
* force polytope `minkowski sum <#pycapacity\.robot\.force_polytope_sum_withfaces>`  and `intersection <#pycapacity\.robot\.force_polytope_intersection>`_

"""

import numpy as np
# minkowski sum
from scipy.spatial import ConvexHull

# import the algos
from pycapacity.algorithms import hyper_plane_shift_method, vertex_enumeration_auctus
from pycapacity.algorithms import *

# velocity manipulability calculation
def velocity_ellipsoid(J, dq_max):
    """
    Velocity manipulability ellipsoid calculation

    .. math:: E_f = \{\dot{x}~ |~ J\dot{q} = \dot{x},\quad ||\dot{q}|| \leq \dot{q}_{max}\}

    Args:
        J: position jacobian
        dq_max:  maximal joint velocities

    Returns
    ---------
        S(list):  
            list of axis lengths
        U(matrix): 
            list of axis vectors
    """ 
    # jacobian calculation
    Jac = J
    # limits scaling
    W = np.diagflat(dq_max)
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # return the singular values and the unit vector angle
    return [S, U]

# acceleration manipulability calculation
def acceleration_ellipsoid(J, M, t_max):
    """
    Acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
   
    .. math:: E_a = \{\ddot{x}~ |~ \ddot{x} = JM^{-1}\\tau,\quad ||{\\tau}|| \leq {\\tau}_{max}\}

    Args:
        J: matrix jacobian
        M: matrix inertia 
        t_max:  maximal joint torques
        
    Returns
    ---------
        S(list):  
            list of axis lengths
        U(matrix): 
            list of axis vectors
    """ 
    # jacobian calculation
    Jac = J.dot(np.linalg.pinv(M))
    # limits scaling
    W = np.linalg.pinv(np.diagflat(t_max))
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # return the singular values and the unit vector angle
    return [S, U]

# force ellipsoid calculation
def force_ellipsoid(J, t_max):
    """
    Force manipulability ellipsoid calculation

    .. math:: E_f = \{f~ |~ \\tau  = J^Tf,\quad ||\\tau|| \leq {\\tau}_{max}\}

    Args:
        J: matrix jacobian
        t_max:  maximal joint torques

    Returns
    ---------
        S(list):  
            list of axis lengths
        U(matrix): 
            list of axis vectors
    """ 
    # jacobian calculation
    Jac = J
    # limits scaling
    W = np.linalg.pinv(np.diagflat(t_max))
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # return the singular values and the unit vector angle
    return [np.divide(1,S), U]

# maximal end effector force
def force_polytope_intersection(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias, t2_bias):
    """
    Force polytope representing the intersection of the capacities of the two robots in certain configurations.


    .. math:: P_f = \{f~ | ~ f_1\! =\! f_2\! =\! f, ~~ \\tau_1  = J_1^Tf_1, ~~ \\tau_2  = J_2^Tf_2, ~~ {t}_{1,min} \leq \\tau_1 \leq {\\tau}_{1,max},~~~ {\\tau}_{2,min} \leq \\tau_1 \leq {\\tau}_{2,max}\}


    Based on the ``vertex_enumeration_auctus`` algorihtm.

    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1
        t2_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2

    Returns
    ---------
        f_vertex(list):  
            vertices of the polytope
        t_vertex : 
            joint torques corresponging to the force vertices
    """
    # jacobian calculation
    Jac =  np.hstack((Jacobian1,Jacobian2))
    t_min = np.hstack((t1_min.flatten(),t2_min.flatten()))
    t_max = np.hstack((t1_max.flatten(),t2_max.flatten()))
    if t1_bias is None:
        t_bias = None
    else:
        t_bias = np.hstack((t1_bias.flatten(), t2_bias.flatten()))

    return force_polytope(Jac, t_max,t_min, t_bias)

def force_polytope_sum_withfaces(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias = None, t2_bias = None):
    """
    Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
    With ordered vertices into the faces.

    .. math:: P_f = \{f~ | ~ f\! =\! f_1\! +\! f_2, ~~ \\tau_1  = J_1^Tf_1, ~~ \\tau_2  = J_2^Tf_2, ~~ {\\tau}_{1,min} \leq \\tau_1 \leq {\\tau}_{1,max},~~~ {\\tau}_{2,min} \leq \\tau_1 \leq {\\tau}_{2,max}\}

    Based on the ``vertex_enumeration_auctus`` algorihtm.

    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1
        t2_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2

    Returns
    ---------
        f_vertex(list):  
            vertices of the polytope
        faces(list): 
            list of vertex indexes belonging to faces
    """ 
    # calculate two polytopes
    f_vertex1, t_vertex1 = force_polytope(Jacobian1, t1_max, t1_min, t1_bias)
    f_vertex2, t_vertex2 = force_polytope(Jacobian2, t2_max, t2_min, t2_bias)
    # then do a minkowski sum
    m, n = Jacobian1.shape
    f_sum = np.zeros((f_vertex1.shape[1]*f_vertex2.shape[1],m))
    for i in range(f_vertex1.shape[1]):
        for j in range(f_vertex2.shape[1]):
            f_sum[i*f_vertex2.shape[1] + j] = np.array(f_vertex1[:,i]+f_vertex2[:,j]).flat

    hull = ConvexHull(f_sum, qhull_options='QJ')
    hull = ConvexHull(hull.points[hull.vertices], qhull_options='QJ')
    return hull.points.T, hull.simplices

# maximal end effector force
def force_polytope(Jacobian, t_max, t_min, t_bias = None):
    """
    Force polytope representing the capacities of the two robots in a certain configuration

    .. math:: P_f = \{f~ |~ \\tau  = J^Tf,\quad {\\tau}_{min} \leq \\tau \leq {\\tau}_{max}\}

    Based on the ``vertex_enumeration_auctus`` algorihtm.

    Args:
        Jacobian:  position jacobian 
        t_max:  maximal joint torques 
        t_min:  minimal joint torques 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 

    Returns
    ---------
        f_vertex : 
            vertices of the polytope
        t_vertex : 
            joint torques corresponging to the force vertices
    """ 

    # jacobian calculation
    return vertex_enumeration_auctus(Jacobian.T, t_max, t_min, t_bias)
    
def force_polytope_withfaces(Jacobian, t_max, t_min, t_bias = None):
    """
    Force polytope representing the capacities of the two robots in a certain configuration.
    With vertices ordered into the faces

    .. math:: P_f = \{f~ |~ \\tau  = J^Tf,\quad {\\tau}_{min} \leq \\tau \leq {\\tau}_{max}\}

    Based on the ``vertex_enumeration_auctus`` algorihtm.

    Args:
        Jacobian:  position jacobian 
        t_max:  maximal joint torques 
        t_min:  minimal joint torques 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces

    Returns
    ---------
        f_vertex(list):  
            vertices of the polytope
        faces(list): 
            list of vertex indexes belonging to faces
    """ 
    force_vertex, t_vertex = force_polytope(Jacobian, t_max, t_min, t_bias)
    
    # find the polytope faces
    polytope_faces = []
    if force_vertex.shape[0] == 1:
        polytope_faces = [0, 1]
    else:        
        try:
            hull = ConvexHull(force_vertex.T, qhull_options='QJ')
            polytope_faces = hull.simplices
        except :
            print("Convex hull error - cannot create faces")
            return force_vertex, []    


    return force_vertex, polytope_faces

def force_polytope_intersection_withfaces(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias=None, t2_bias=None):
    """
    Force polytope representing the intersection of the capacities of the two robots in certain configurations.
    With ordered vertices into the faces.

    .. math:: P_f = \{f~ | ~ f_1\! =\! f_2\! =\! f, ~~ \\tau_1  = J_1^Tf_1, ~~ \\tau_2  = J_2^Tf_2, ~~ {\\tau}_{1,min} \leq \\tau_1 \leq {\\tau}_{1,max},~~~ {\\tau}_{2,min} \leq \\tau_1 \leq {\\tau}_{2,max}\}
    
    Based on the ``vertex_enumeration_auctus`` algorihtm.

    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1
        t2_bias:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2

    Returns
    ---------
        f_vertex(list):  
            vertices of the polytope
        faces(list): 
            list of vertex indexes belonging to faces
    """
    force_vertex, t_vertex = force_polytope_intersection(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias, t2_bias)

    print(force_vertex.shape)
    polytope_faces = []
    if force_vertex.shape[0] == 1:
        polytope_faces = [0, 1]
    else:        
        hull = ConvexHull(force_vertex.T, qhull_options='QJ')
        polytope_faces = hull.simplices

    return force_vertex, polytope_faces


def velocity_polytope(Jacobian, dq_max, dq_min):
    """
    Velocity polytope calculating function

    .. math:: P_f = \{\dot{x}~ |~ J\dot{q} = \dot{x},\quad {\dot{q}}_{min} \leq \dot{q} \leq {\dot{q}}_{max}\}

    Based on the ``hyper_plane_shifting_method`` algorihtm.

    Args:
        Jacobian:  position jacobian 
        dq_max:  maximal joint velocities 
        dq_min:  minimal joint velocities 

    Returns:
        velocity_vertex(list):  vertices of the polytope
    """ 
    H, d = hyper_plane_shift_method(Jacobian,dq_min,dq_max)
    velocity_vertex, vel_faces = hspace_to_vertex(H,d)
    return velocity_vertex

def velocity_polytope_withfaces(Jacobian, dq_max, dq_min):
    """
    Velocity polytope calculating function, with faces

    .. math:: P_f = \{\dot{x}~ |~ J\dot{q} = \dot{x},\quad {\dot{q}}_{min} \leq \dot{q} \leq {\dot{q}}_{max}\}

    Based on the ``hyper_plane_shifting_method`` algorihtm.

    Args:
        Jacobian:  position jacobian 
        dq_max:  maximal joint velocities 
        dq_min:  minimal joint velocities 

    Returns
    ---------
        velocity_vertex(list):  
            vertices of the polytope
        faces(list): 
            list of vertex indexes belonging to faces
    """ 
    H, d = hyper_plane_shift_method(Jacobian,dq_min,dq_max)
    velocity_vertex, vel_faces = hspace_to_vertex(H,d)
    return velocity_vertex, vel_faces

def acceleration_polytope(J, M, t_max, t_min, t_bias= None):
    """
    Acceleration polytope calculating function

    .. math:: P_a = \{\ddot{x}~ |~ \ddot{x} = JM^{-1}\\tau,\quad {\\tau}_{min} \leq \\tau \leq {\\tau}_{max}\} 

    Based on the ``hyper_plane_shifting_method`` algorihtm.

    Args:
        J:  position jacobian 
        M:  inertia matrix 
        t_max:  maximal joint torque 
        t_min:  minimal joint torque 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces

    Returns:
        acceleration_vertex(list):  vertices of the polytope
    """ 
    B = J.dot(np.linalg.pinv(M))
    if t_bias is not None:
        t_min = t_min - t_bias
        t_max = t_max - t_bias
    
    H, d = hyper_plane_shift_method(B,t_min, t_max)
    vertex, faces = hspace_to_vertex(H,d)
    return vertex

def acceleration_polytope_withfaces(J, M, t_max, t_min, t_bias= None):
    """
    Acceleration polytope calculating function

    
    .. math:: P_a = \{\ddot{x}~ |~ \ddot{x} = JM^{-1}\\tau,\quad {\\tau}_{min} \leq \\tau \leq {\\tau}_{max}\} 

    Based on the ``hyper_plane_shifting_method`` algorihtm.

    Args:
        J:  position jacobian 
        M:  inertia matrix 
        t_max:  maximal joint torque 
        t_min:  minimal joint torque 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces
    
    Returns
    ---------
        acceleration_vertex(list):  
            vertices of the polytope
        faces(list):
            list of vertex indexes belonging to faces
    """ 
    B = J.dot(np.linalg.pinv(M))
    if t_bias is not None:
        t_min = t_min - t_bias
        t_max = t_max - t_bias
    
    H, d = hyper_plane_shift_method(B,t_min, t_max)
    vertex, faces = hspace_to_vertex(H,d)
    return vertex, faces

def reachable_space_approximation_polytope( M, J, q0, horizon, t_max,t_min, t_bias= None, q_max= None,q_min= None, dq_max= None,dq_min= None, tolerance=1e-3, max_iteration=500, verbose=True):
    """
    Reachable space aproximation function based on convex polytopes. For a given time horizon, it calculates the reachable space of the robot.
    It evaluates the polytope of a form:
    
    .. math:: P_x = \{\Delta x~ |~ \Delta{x} = JM^{-1}\\tau \Delta t_{h}^2/2,
    .. math:: {\\tau}_{min} - \\tau_{bias} \leq \\tau \leq {\\tau}_{max} - \\tau_{bias}
    .. math::  \dot{q}_{min} \leq JM^{-1}\\tau \Delta t_{h}  \leq \dot{q}_{max}
    .. math::  {q}_{min} \leq JM^{-1}\\tau \Delta t_{h}^2/2  \leq {q}_{max} \}

    where :math:`\\tau_{bias}` is the bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces which is optional.
    and :math:`\Delta t_{h}` is the time horizon. If limits on joint velocity :math:`\dot{q}_{min}` and :math:`\dot{q}_{max}` or joint postion limits :math:`{q}_{min}` and :math:`{q}_{max}` are not given, the function calculates the ploytope  without them.

    Based on the ``iterative_convex_hull`` algorihtm.

    Args:
        M:  inertia matrix
        J:  position jacobian
        q0:  initial joint position
        horizon:  time horizon
        t_max:  maximal joint torque 
        t_min:  minimal joint torque
        t_bias:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces (optional)
        q_max:  maximal joint position (optional)
        q_min:  minimal joint position (optional)
        dq_max:  maximal joint velocities (optional)
        dq_min:  minimal joint velocities (optional)
        tolerance:  tolerance of the algorithm (optional)
        max_iteration:  maximal number of iterations (optional)
        verbose:  verbose mode (optional)
    Returns
    ---------
        vertex(list):  
            vertices of the polytope
        faces(list):  
            list of vertex indexes belonging to faces
        t_vert(list):  
            vertices of the torque polytope
        H(list):  
            list of hyperplanes - Hx<=d
        d(list):  
            list of hyperplanes' offsets - Hx<=d

    Note:
        *Skuric, Antun, Vincent Padois, and David Daney. "Approximating robot reachable space using convex polytopes." Human-Friendly Robotics 2022: HFR: 15th International Workshop on Human-Friendly Robotics. Cham: Springer International Publishing, 2023.*

    """
    # jacobian (only position part)
    Jac = J
    # mass matrx
    M_inv = np.linalg.pinv(M)

    if t_bias is not None:
        t_min = t_min - t_bias
        t_max = t_max - t_bias

    G_in = None
    if dq_max is not None and dq_min is not None:
        G_in = np.vstack((
            M_inv*horizon,   
            -M_inv*horizon))
        h_in = np.hstack((
            dq_max.flatten(),
            -dq_min.flatten()))
    
    if q_max is not None and q_min is not None:
        if G_in is not None:
            G_in = np.vstack((G_in,
                M_inv*horizon**2/2,
                -M_inv*horizon**2/2))
            h_in = np.hstack((h_in,
                (q_max.flatten()-q0),
                -(q_min.flatten()-q0)))
        else:
            G_in = np.vstack((
                M_inv*horizon**2/2,
                -M_inv*horizon**2/2))
            h_in = np.hstack((
                (q_max.flatten()-q0),
                -(q_min.flatten()-q0)))
    

    vertex, H,d, faces_index, t_vert, x_vert =  iterative_convex_hull_method(
        A = np.eye(J.shape[0]),
        B = np.dot(Jac, M_inv)*horizon**2/2,
        y_max = t_max, 
        y_min = t_min,
        G_in = G_in,
        h_in = h_in,
        tol = tolerance,
        max_iter=max_iteration,
        verbose=True)
    
    return vertex, faces_index, t_vert, H, d