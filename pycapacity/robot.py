"""
Overview
---------

This is a pyhton module which implements different robot performance metrics based on polytopes and ellipsoids.

* acceleration `polytope <#pycapacity\.robot\.acceleration_polytope>`_ and `ellipsoid <#pycapacity\.robot\.acceleration_ellipsoid>`_
* velocity (manipulability) `polytope <#pycapacity\.robot\.velocity_polytope>`_ and `ellipsoid <#pycapacity\.robot\.velocity_ellipsoid>`_
* force `polytope <#pycapacity\.robot\.force_polytope>`_ and `ellipsoid <#pycapacity\.robot\.force_ellipsoid>`_
* force polytope `minkowski sum <#pycapacity\.robot\.force_polytope_sum>`_  and `intersection <#pycapacity\.robot\.force_polytope_intersection>`_
* reachable space approximation `polytope <#pycapacity\.robot\.reachable_space_approximation>`_

"""

import numpy as np
# minkowski sum
from scipy.spatial import ConvexHull

# import the algos
from pycapacity.algorithms import hyper_plane_shift_method, vertex_enumeration_auctus
from pycapacity.algorithms import *

from pycapacity.objects import *

def velocity_ellipsoid(J, dq_max):
    """
    Velocity manipulability ellipsoid calculation

    .. math:: E_f = \{\dot{x}~ |~ J\dot{q} = \dot{x},\quad ||\dot{q}|| \leq \dot{q}_{max}\}

    Args:
        J: position jacobian
        dq_max:  maximal joint velocities

    Returns
    ---------
        ellipsoid(Ellipsoid):
            ellipsoid object with the following attributes ``radii``, ``axes``
    """ 
    # jacobian calculation
    Jac = J
    # limits scaling
    W = np.diagflat(dq_max)
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))

    # create the ellipsoid from the singular values and the unit vector angle
    ellipsoid = Ellipsoid(radii=S, rotation=U)
    return ellipsoid

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
        ellipsoid(Ellipsoid):
            ellipsoid object with the following attributes ``radii``, ``axes``
    """ 
    # jacobian calculation
    Jac = J.dot(np.linalg.pinv(M))
    # limits scaling
    W = np.linalg.pinv(np.diagflat(t_max))
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # create the ellipsoid from the singular values and the unit vector angle
    ellipsoid = Ellipsoid(radii=S, rotation=U)
    return ellipsoid

def force_ellipsoid(J, t_max):
    """
    Force manipulability ellipsoid calculation

    .. math:: E_f = \{f~ |~ \\tau  = J^Tf,\quad ||\\tau|| \leq {\\tau}_{max}\}

    Args:
        J: matrix jacobian
        t_max:  maximal joint torques

    Returns
    ---------
        ellipsoid(Ellipsoid):
            ellipsoid object with the following attributes ``radii``, ``axes``
    """ 
    # jacobian calculation
    Jac = J
    # limits scaling
    W = np.linalg.pinv(np.diagflat(t_max))
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # create the ellipsoid from the singular values and the unit vector angle
    ellipsoid = Ellipsoid(radii=np.divide(1,S), rotation=U)
    return ellipsoid

def force_polytope_intersection(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias=None, t2_bias=None, options = None):
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
        options:  dictionary of options for the algorithm (currently supported: ``calculate_faces``)
    Returns
    ---------
        polytope(Polytope):
            force polytope representing the intersection of the capacities of the two robots in certain configurations. Vertex representation ``vertices`` and hal-plane representation ``H`` and ``d`` (face representation ``faces`` and ``face_indices`` if ``calculate_faces`` is set to ``True`` in ``options``)
    """
    # jacobian calculation
    Jac =  np.hstack((Jacobian1,Jacobian2))
    t_min = np.hstack((t1_min.flatten(),t2_min.flatten()))
    t_max = np.hstack((t1_max.flatten(),t2_max.flatten()))
    if t1_bias is None:
        t_bias = None
    else:
        t_bias = np.hstack((t1_bias.flatten(), t2_bias.flatten()))

    return force_polytope(Jac, t_max, t_min, t_bias, options=options)

def force_polytope_sum(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias = None, t2_bias = None, options = None):
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
        options: dictionary of additional options (currently supported only ``calculate_faces`` option)

    Returns
    ---------
        polytope(Polytope):
            polytope object with ``vertices`` and half-plane representation ``H`` and ``d``, ( ``faces`` and ``face_indices`` if ``calculate_faces`` option is set to True)
    """ 
    # calculate two polytopes
    f1_poly = force_polytope(Jacobian1, t1_max, t1_min, t1_bias)
    f2_poly = force_polytope(Jacobian2, t2_max, t2_min, t2_bias)
    f_vertex1 = f1_poly.vertices
    f_vertex2 = f2_poly.vertices
    # then do a minkowski sum
    m, n = Jacobian1.shape
    f_sum = np.zeros((f_vertex1.shape[1]*f_vertex2.shape[1],m))
    for i in range(f_vertex1.shape[1]):
        for j in range(f_vertex2.shape[1]):
            f_sum[i*f_vertex2.shape[1] + j] = np.array(f_vertex1[:,i]+f_vertex2[:,j]).flat

    hull = ConvexHull(f_sum, qhull_options='QJ')
    poly = Polytope(vertices = hull.points[hull.vertices].T, H=hull.equations[:,:-1], d=-hull.equations[:,-1])

    # check if the faces should be calculated
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] == True:
        poly.find_faces()    

    return poly

def force_polytope(Jacobian, t_max, t_min, t_bias = None, options = None):
    """
    Force polytope representing the capacities of the two robots in a certain configuration

    .. math:: P_f = \{f~ |~ \\tau  = J^Tf,\quad {\\tau}_{min} \leq \\tau \leq {\\tau}_{max}\}

    Based on the ``vertex_enumeration_auctus`` algorihtm.

    Args:
        Jacobian:  position jacobian 
        t_max:  maximal joint torques 
        t_min:  minimal joint torques 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 
        options: dictionary of options for the polytope calculation - currently only ``calculate_faces`` is supported

    Returns
    ---------
        force_polytope(Polytope):
            polytope object with ``vertices``, torque vertices ``torque_vertices`` (``face_indices`` and ``faces`` if option ``calculate_faces`` is set to True)
    """ 

    # jacobian calculation
    f_vert, t_vert = vertex_enumeration_auctus(Jacobian.T, t_max, t_min, t_bias)

    # create polytope
    poly = Polytope(vertices=f_vert)
    poly.torque_vertices = t_vert

    # if faces are required
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] == True:
        poly.face_indices = vertex_to_faces(vertex=f_vert)
        poly.faces = face_index_to_vertex(poly.vertices, poly.face_indices)

    return poly

def velocity_polytope(Jacobian, dq_max, dq_min, options = None):
    """
    Velocity polytope calculating function

    .. math:: P_f = \{\dot{x}~ |~ J\dot{q} = \dot{x},\quad {\dot{q}}_{min} \leq \dot{q} \leq {\dot{q}}_{max}\}

    Based on the ``hyper_plane_shifting_method`` algorihtm.

    Args:
        Jacobian:  position jacobian 
        dq_max:  maximal joint velocities 
        dq_min:  minimal joint velocities 
        options: dictionary of options for the polytope calculation - currently only ``calculate_faces`` is supported

    Returns:
        velocity_polytope(Polytope):
            polytope object with ``vertices``, halfspaces ``H`` and ``d`` (``face_indices`` and ``faces`` if option ``calculate_faces`` is set to True)
    """ 
    H, d = hyper_plane_shift_method(Jacobian,dq_min,dq_max)
    velocity_vertex, vel_faces = hspace_to_vertex(H,d)

    # create polytope
    poly = Polytope(vertices=velocity_vertex, H=H, d=d)

    # if faces are required
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] == True:
        poly.face_indices = vel_faces
        poly.faces = face_index_to_vertex(poly.vertices, poly.face_indices)

    return poly

def acceleration_polytope(J, M, t_max, t_min, t_bias= None, options = None):
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
        options: dictionary of options for the polytope calculation (currently only ``calculate_faces`` is supported)

    Returns:
        acceleration_polytope(Polytope):
            polytope object with ``vertices``, half-plane representation with matrices``H`` and ``d`` (``face_indices`` and ``faces`` if option ``calculate_faces`` is set to True)
    
    """ 
    B = J.dot(np.linalg.pinv(M))
    if t_bias is not None:
        t_min = t_min - t_bias
        t_max = t_max - t_bias
    
    H, d = hyper_plane_shift_method(B,t_min, t_max)
    vertex, faces = hspace_to_vertex(H,d)


    # create polytope
    poly = Polytope(vertices=vertex, H=H, d=d)

    # if faces are required
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] == True:
        poly.face_indices = faces
        poly.faces = face_index_to_vertex(poly.vertices, poly.face_indices)
    
    return poly

def reachable_space_approximation( M, J, q0, horizon, t_max,t_min, t_bias= None, q_max= None,q_min= None, dq_max= None,dq_min= None, options= None):
    """
    Reachable space aproximation function based on convex polytopes. For a given time horizon, it calculates the reachable space of the robot.
    It evaluates the polytope of a form:
    
    .. math:: P_x = \{\Delta x~ |~ \Delta{x} = JM^{-1}\\tau \Delta t_{h}^2/2,
    .. math:: {\\tau}_{min} - \\tau_{bias} \leq \\tau \leq {\\tau}_{max} - \\tau_{bias}
    .. math::  \dot{q}_{min} \leq M^{-1}\\tau \Delta t_{h}  \leq \dot{q}_{max}
    .. math::  {q}_{min} \leq M^{-1}\\tau \Delta t_{h}^2/2  \leq {q}_{max} \}

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
        options: dictionary of options for the polytope calculation - currently supported ``tolerance``, ``max_iteration`` and ``verbose`` (default: ``tolerance=1e-3``, ``max_iteration=500``, ``verbose=False``)
    
    Returns
    ---------
        polytope(Polytope):
            polytope object with ``vertices``, torque vertices ``torque_vertices``, half-plane representation with matrices ``H`` and ``d``, and faces defiend by ``face_indices`` and ``faces`` 


    :raises ValueError: if the mass and jacobian matrices are not appropriate size
    :raises ValueError: if any of the provided joint limits sizes are not equal to the number of joints
    :raises ValueError: if bias joint torques are given and their size is not equal to the number of joints


    Note:
        *Skuric, Antun, Vincent Padois, and David Daney. "Approximating robot reachable space using convex polytopes." Human-Friendly Robotics 2022: HFR: 15th International Workshop on Human-Friendly Robotics. Cham: Springer International Publishing, 2023.*

    """
    # if options are given
    if options is not None and 'tolerance' in options.keys():
        tolerance = options['tolerance']
    else:
        tolerance = 1e-3
    if options is not None and 'max_iteration' in options.keys():
        max_iteration = options['max_iteration']
    else:
        max_iteration = 500
    if options is not None and 'verbose' in options.keys():
        verbose = options['verbose']
    else:
        verbose = False

    # jacobian (only position part)
    Jac = J
    m,n = Jac.shape
    # mass matrx
    M_inv = np.linalg.pinv(M)

    # check if matrices have the right dimensions
    if n != M.shape[0]:
        raise ValueError('Jacobian and mass matrix have different number of columns {} and {}'.format(n, M.shape[0]))
    # check if limits have good dimensions
    if t_max.shape[0] != n:
        raise ValueError('t_max has wrong dimensions {}, should be {}'.format(t_max.shape[0], n))
    # check for joint position limits as well
    if q_max is not None and q_max.shape[0] != n:
        raise ValueError('q_max has wrong dimensions {}, should be {}'.format(q_max.shape[0], n))
    # joint velocity limits
    if dq_max is not None and dq_max.shape[0] != n:
        raise ValueError('dq_max has wrong dimensions {}, should be {}'.format(dq_max.shape[0], n))
    # bias not the good size
    if t_bias is not None and t_bias.shape[0] != n:
        raise ValueError('t_bias has wrong dimensions {}, should be {}'.format(t_bias.shape[0], n))


    # if bias is given
    if t_bias is not None:
        t_min = t_min - t_bias
        t_max = t_max - t_bias

    # initial value for inequality constraints
    G_in, h_in = None, None

    # if limits on joint velocity are given
    if dq_max is not None and dq_min is not None:
        G_in = np.vstack((
            M_inv*horizon,   
            -M_inv*horizon))
        h_in = np.hstack((
            dq_max.flatten(),
            -dq_min.flatten()))
    
    # if limits on joint position are given
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
    
    # calculate the polytope
    vertex, H,d, faces_index, t_vert, x_vert =  iterative_convex_hull_method(
        A = np.eye(J.shape[0]),
        B = np.dot(Jac, M_inv)*horizon**2/2,
        y_max = t_max, 
        y_min = t_min,
        G_in = G_in,
        h_in = h_in,
        tol = tolerance,
        max_iter=max_iteration,
        verbose=verbose)
    
    # construct a polytope object
    poly = Polytope(vertices=vertex, H=H, d=d, face_indices=faces_index)
    if options and 'calculate_faces' in options.keys() and options['calculate_faces']:
        poly.face_indices = faces_index
        poly.faces = face_index_to_vertex(poly.vertices, poly.face_indices)
    poly.torque_vertices = t_vert
    return poly