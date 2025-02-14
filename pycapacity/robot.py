"""
Overview
---------

This is a python module which implements different robot performance metrics based on polytopes and ellipsoids.

* acceleration `polytope <#pycapacity\.robot\.acceleration_polytope>`_ and `ellipsoid <#pycapacity\.robot\.acceleration_ellipsoid>`_
* velocity (manipulability) `polytope <#pycapacity\.robot\.velocity_polytope>`_ and `ellipsoid <#pycapacity\.robot\.velocity_ellipsoid>`_
* force `polytope <#pycapacity\.robot\.force_polytope>`_ and `ellipsoid <#pycapacity\.robot\.force_ellipsoid>`_
* force polytope `minkowski sum <#pycapacity\.robot\.force_polytope_sum>`_  and `intersection <#pycapacity\.robot\.force_polytope_intersection>`_
* reachable space approximation `polytope <#pycapacity\.robot\.reachable_space_approximation>`_
* reachable space `nonlinear <#pycapacity\.robot\.reachable_space_nonlinear>`_

"""

import numpy as np
# minkowski sum
from scipy.spatial import ConvexHull
from scipy.linalg import block_diag

# import the algos
from pycapacity.algorithms import hyper_plane_shift_method, vertex_enumeration_vepoli2, stack
from pycapacity.algorithms import *

from pycapacity.objects import *

# check if CGAL is installed
try:
    from CGAL.CGAL_Alpha_wrap_3 import *
    from CGAL.CGAL_Kernel import *
    from CGAL.CGAL_Polyhedron_3 import Polyhedron_3
    from CGAL.CGAL_Mesh_3 import *
    CGAL_INSTALLED = True
except ImportError:
    CGAL_INSTALLED = False

def velocity_ellipsoid(J, dq_max):
    """
    Velocity manipulability ellipsoid calculation

    .. math:: E_f = \{\dot{x}~ |~ J\dot{q} = \dot{x},\quad ||W^{-1}\dot{q}|| \leq 1\}

    where

    .. math:: W=diag(\dot{q}_{max})

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
   
    .. math:: E_a = \{\ddot{x}~ |~ \ddot{x} = JM^{-1}\\tau,\quad ||W^{-1}{\\tau}|| \leq 1\}

    where

    .. math:: W=diag(\\tau_{max})

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
    W = np.diagflat(t_max)
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # create the ellipsoid from the singular values and the unit vector angle
    ellipsoid = Ellipsoid(radii=S, rotation=U)
    return ellipsoid

def force_ellipsoid(J, t_max):
    """
    Force manipulability ellipsoid calculation

    .. math:: E_f = \{f~ |~ \\tau  = J^Tf,\quad ||W^{-1}\\tau|| \leq 1\}

    where

    .. math:: W=diag(\\tau_{max})

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


    Based on the ``vertex_enumeration_vepoli2`` algorithm.

    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias: bias joint torques due to the gravity, robot dynamics and maybe some already applied forces for robot 1
        t2_bias: bias joint torques due to the gravity, robot dynamics and maybe some already applied forces for robot 2
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

    Based on the ``vertex_enumeration_vepoli2`` algorithm.

    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias: bias joint torques due to the gravity, robot dynamics and maybe some already applied forces for robot 1
        t2_bias: bias joint torques due to the gravity, robot dynamics and maybe some already applied forces for robot 2
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

    Based on the ``vertex_enumeration_vepoli2`` algorithm.

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
    f_vert, t_vert = vertex_enumeration_vepoli2(Jacobian.T, t_max, t_min, t_bias)

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

    Based on the ``hyper_plane_shifting_method`` algorithm.

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

    Based on the ``hyper_plane_shifting_method`` algorithm.

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

def reachable_space_approximation( M, J, q0, horizon, t_max,t_min, t_bias= None, q_max= None,q_min= None, dq_max= None,dq_min= None, x0=None, A_x=None, b_x =None, options= None):
    """
    Reachable space aproximation function based on convex polytopes. For a given time horizon, it calculates the reachable space of the robot.
    It evaluates the polytope of a form:
    
    .. math:: P_x = \{\Delta x~ |~ \Delta{x} = JM^{-1}\\tau \Delta t_{h}^2/2,
    .. math:: {\\tau}_{min} - \\tau_{bias} \leq \\tau \leq {\\tau}_{max} - \\tau_{bias}
    .. math::  \dot{q}_{min} \leq M^{-1}\\tau \Delta t_{h}  \leq \dot{q}_{max}
    .. math::  {q}_{min} \leq M^{-1}\\tau \Delta t_{h}^2/2  \leq {q}_{max} 
    .. math::  A_x \Delta{x} \leq b_x - A_x x_0\}

    where :math:`\\tau_{bias}` is the bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces which is optional.
    and :math:`\Delta t_{h}` is the time horizon. If limits on joint velocity :math:`\dot{q}_{min}` and :math:`\dot{q}_{max}` or joint postion limits :math:`{q}_{min}` and :math:`{q}_{max}` are not given, the function calculates the ploytope  without them.
    `A_x` and `b_x` are additional inequality covex constraints in the Cartesian position space. `x0` is the initial Cartesian position corresponding to the initial joint position `q0`.

    Based on the ``iterative_convex_hull`` algorithm.

    Args:
        M:  inertia matrix
        J:  position jacobian
        q0:  initial joint position
        horizon:  time horizon
        t_max:  maximal joint torque 
        t_min:  minimal joint torque
        x0:  initial Cartesian position (optional)
        t_bias:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces (optional)
        q_max:  maximal joint position (optional)
        q_min:  minimal joint position (optional)
        dq_max:  maximal joint velocities (optional)
        dq_min:  minimal joint velocities (optional)
        A_x, b_x:  additional inequality constraints matrices in Cartesian position space (optional)
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
    
    
    if A_x is not None and b_x is not None:
        if x0 is not None:
            b_x = b_x - A_x@x0
        if G_in is not None:
            G_in = np.vstack((G_in, A_x@Jac@M_inv*horizon**2/2))
            h_in = np.hstack((h_in, b_x))
        else:
            G_in = A_x*Jac*M_inv*horizon**2/2
            h_in = b_x

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
    if x0 is not None:
        x0 = x0.reshape(-1,1)
    else:
        x0 = np.zeros((J.shape[0],1))
    poly = Polytope(vertices=vertex + x0, H=H, d=d, face_indices=faces_index)
    if options and 'calculate_faces' in options.keys() and options['calculate_faces']:
        poly.face_indices = faces_index
        poly.faces = face_index_to_vertex(poly.vertices, poly.face_indices)
    poly.torque_vertices = t_vert
    return poly

import numpy.matlib


# conditionally implement a funciton (if CGAL is installed)
if CGAL_INSTALLED:
    def alpha_shape_with_cgal(coords, alpha=None):
        """
        Compute the alpha shape of a set of points.
        Retrieved from http://blog.thehumangeo.com/2014/05/12/drawing-boundaries-in-python/

        :param coords : Coordinates of points
        :param alpha: List of alpha values to influence the gooeyness of the border. Smaller numbers don't fall inward as much as larger numbers. 
        Too large, and you lose everything!
        :return: vertices and faces of the alpha shape
        """
        if alpha is None:
            bbox_diag = np.linalg.norm(np.max(coords,0)-np.min(coords,0))
            alpha_value = bbox_diag/5
        else:
            alpha_value = np.mean(alpha)
        # Convert to CGAL point
        points = [Point_3(pt[0], pt[1], pt[2]) for pt in coords]
        # Compute alpha shape
        Q = Polyhedron_3()
        a = alpha_wrap_3(points,alpha_value,0.01,Q)
        #Q.make_tetrahedron()
        alpha_shape_vertices = np.array([(vertex.point().x(), vertex.point().y(), vertex.point().z()) for vertex in Q.vertices()])
        alpha_shape_faces = np.array([
            np.array([
                (face.halfedge().vertex().point().x(), face.halfedge().vertex().point().y(), face.halfedge().vertex().point().z()),
                (face.halfedge().next().vertex().point().x(), face.halfedge().next().vertex().point().y(), face.halfedge().next().vertex().point().z()),
                (face.halfedge().next().next().vertex().point().x(), face.halfedge().next().next().vertex().point().y(), face.halfedge().next().next().vertex().point().z())
                #for i in face.halfedge()
            ])
            for face in Q.facets()])
                
        return alpha_shape_vertices,alpha_shape_faces


# reachable space calculation algorithm
def reachable_space_nonlinear(forward_func, q0, time_horizon, q_max, q_min, dq_max, dq_min, options=None):

    """
    Compute the reachable set of the robot for the given joint configuration.
    The algorithm calculates the reachable set of cartesian position of the desired frame of the robot given the robots joint position and joint velocity limits.
    The output of the algorithm is the reachable space that the robot is able to reach within the horizon time, while guaranteeing that the joint position and velocity limits are not violated.
    
    If you are interested in the complete workspace of the robot, you can set a large time horizon (>1 second)
    
    .. math:: C_x = \{ x~ |~ x = f_{fk}(q_0 + \dot{q}\Delta t), 
    .. math::  \dot{q}_{min} \leq \dot{q}  \leq \dot{q}_{max},\quad {q}_{min} \leq q_0 + \dot{q}\Delta t  \leq {q}_{max} \}

    The parameters of the algorithm are set using the options dictionary. The following options are available:
    
    * n_samples: The number of samples to use for the discretization of the joint velocity space. The higher the number of samples, the more accurate the reachable set will be, however the longer the computation time will be
    * facet_dim: The dimension of the facet that will be sampled. Between 0 and the number of DOF of the robot.  The higher the number of samples, the more accurate the reachable set will be, however the longer the computation time will be
    * convex_hull: Approximate the reachable set with a convex hull (True) or with a non-convex shape (False) - if False, CGAL must be installed
    
    Args:
        forward_func: The forward kinematic function, taking in the current joint position and ouputting the Cartesian space position (no orientation)
        q0: Current joint configuration
        time_horizon: The time horizon for which to compute the reachable set
        q_max:  maximal joint position
        q_min:  minimal joint position
        dq_max:  maximal joint velocities
        dq_min:  minimal joint velocities 
        options: dictionary of options for the polytope calculation - currently supported calculate_faces, n_samples, facet_dim
        
    Returns
    ---------
        polytope(Polytope):
            polytope object with ``vertices`` and faces if option ``calculate_faces`` is set to True in ``options``
    """

    delta_t = time_horizon
    
    
    if 'convex_hull' not in options.keys():
        options['convex_hull'] = True

    n_samples = options['n_samples']
    n_steps = 1
    n_combs = options['facet_dim']

    if len(dq_min) != len(q_min):
        n = len(dq_max)
        dq_max = np.hstack((dq_max, -np.ones(len(q_min)-n)*1000))
        dq_min = np.hstack((dq_min, np.ones(len(q_min)-n)*1000))
      
    n = len(dq_max)  
    n_dof = n_steps*n
    dt = delta_t/n_steps
    
    dq_ub = np.matlib.repmat(np.minimum(dq_max,(q_max.flatten()-q0)/dt), n_steps,1).flatten()
    dq_lb = np.matlib.repmat(np.maximum(dq_min,(q_min.flatten()-q0)/dt), n_steps,1).flatten()

    Dq_ub = np.diag(dq_ub)
    Dq_lb = np.diag(dq_lb)
    sum_steps = np.matlib.repmat(np.eye(n), n_steps,1)

    combs = list(itertools.combinations(range(n_dof), n_combs) )
    perm_set = list(itertools.product([1, 0], repeat=n_dof-n_combs) )

    dq_curve_v = []

    x_rng = np.arange(0, 1, 1/n_samples)
    mat_rng = np.array(list(itertools.product(x_rng, repeat=n_combs))).T
    n_rng = len(mat_rng.T)

    for c in combs:
        c= np.array(c)
        ind = np.ones(n_dof, dtype=bool)
        ind[c] = np.zeros(len(c))
        ind_i = np.argwhere(ind > 0)

        n_ps = len(perm_set)
        ps = np.array(perm_set).T
        dq_i = np.zeros((n_dof,n_ps))
        dq_i[ind,:] = ps

        DQ_i = np.matlib.repmat(dq_i.T, n_rng,1).T
        DQ_i[ind,:] = DQ_i[ind,:]*Dq_ub[ind_i,ind_i] + (1-DQ_i[ind,:])*Dq_lb[ind_i,ind_i] 
        mat = np.diag([dq_ub[c_i]-dq_lb[c_i] for c_i in c ])@mat_rng + np.array([dq_lb[c_i] for c_i in c ])[:,None]
        DQ_i[c,:] = np.matlib.repeat(mat,n_ps,1)
        dq_curve_v = stack(dq_curve_v,DQ_i.T)

    dq_curve_v= np.unique(dq_curve_v,axis=0)
    q_v =(np.array(q0)[:, None] + (dq_curve_v@sum_steps).T*dt).T
    x_curves = np.array([forward_func(q).flatten() for q in q_v])

    if options['convex_hull'] == True:
        poly = Polytope(x_curves.T)
        if options["calculate_faces"]:
            poly.find_faces()
    if options["convex_hull"] == False:
        if CGAL_INSTALLED:
            if options is not None and "alpha" in options.keys():
                vert, faces = alpha_shape_with_cgal(x_curves, options['alpha'])
            else:
                vert, faces = alpha_shape_with_cgal(x_curves)
            vert = faces.reshape(-1,3)
            poly = Polytope(vertices=vert.T, faces=faces)
            poly.face_indices = np.arange(len(vert)).reshape(-1,3)
        else:
            raise ValueError("CGAL is not installed, please install it to use the non-convex option")
    return poly