"""
Overview
---------

This is a pyhton module which implements different human performance metrics based on their musculoskeletal models

* `joint torque polytope <#pycapacity\.human\.joint_torques_polytope>`_
* `acceleration polytope <#pycapacity\.human\.acceleration_polytope>`_
* `force polytope <#pycapacity\.human\.force_polytope>`_
* `velocity polytope <#pycapacity\.human\.velocity_polytope>`_

"""

import numpy as np
from cvxopt import matrix
import cvxopt.glpk

# import the algos
from pycapacity.algorithms import iterative_convex_hull_method, hyper_plane_shift_method
from pycapacity.algorithms import *
from pycapacity.objects import *
import pycapacity.robot as robot

def joint_torques_polytope(N, F_min, F_max, tol=1e-5, options=None):
    """
    A function calculating the polytopes of achievable joint torques
    based on the moment arm matrix `N` :
    
    .. math:: P_{t} = \{ \\tau ~ | ~ \\tau= NF, \quad F_{min} \leq F \leq F_{max}\}

    Based on the ``hyper_plane_shifting_method`` algorihtm.

    Args:
        N: moment arm matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tolerance: tolerance for the polytope calculation
        options: dictionary of options for the polytope calculation (currently only ``calculate_faces`` is supported)

    Returns
    --------
        poly(Polytope):
            polytope object with the following attributes ``vertices``, half-plane representation ``H``, ``d``, (and ``face_indices`` and ``faces`` if option ``calculate_faces`` is set to ``True``)
    """
    H, d = hyper_plane_shift_method(N, F_min, F_max)
    vert, faces = hspace_to_vertex(H,d)

    # create the polytope object
    poly = Polytope(vertices=vert, H=H, d=d)
    # calculate the faces if requested
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] is True:
        poly.face_indices = faces
        poly.faces = face_index_to_vertex(poly.vertices, faces)
    return poly
    
def acceleration_polytope(J, N, M, F_min, F_max, tol=1e-5, options=None):
    """
    A function calculating the polytopes of achievable accelerations
    based on the jacobian matrix `J`, moment arm matrix `N` and mass matrix `M`

    .. math:: P_{a} = \{ \ddot{x} ~ | ~ \ddot{x} = JM^{-1}NF, \quad F_{min} \leq F \leq F_{max}\}

    Based on the ``hyper_plane_shifting_method`` algorihtm.

    Args:
        J: jacobian matrix
        N: moment arm matrix
        M: mass matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tolerance: tolerance for the polytope calculation
        options: dictionary of options for the polytope calculation (currently only ``calculate_faces`` is supported)
        
    Returns
    ---------
        poly(Polytope):
            polytope object with the following attributes ``vertices``, half-plane representation ``H``, ``d``, (and ``face_indices`` and ``faces`` if option ``calculate_faces`` is set to ``True``)
    """
    H,d = hyper_plane_shift_method(J.dot(np.linalg.inv(M).dot(N)),F_min,F_max)
    vert, faces = hspace_to_vertex(H,d)

    # construct polytope object
    poly = Polytope(vertices=vert, H=H, d=d)
    # calculate faces if requested
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] is True:
        poly.face_indices = faces
        poly.faces = face_index_to_vertex(poly.vertices, faces)
    return poly

def force_polytope(J, N, F_min, F_max, tol, torque_bias=None, options=None):
    """
    A function calculating the polytopes of achievable foreces based 
    on the jacobian matrix `J` and moment arm matrix `N`

    .. math:: P_{f} = \{ f ~ | ~ J^Tf = NF, \quad F_{min} \leq F \leq F_{max}\}

    optionally an additional bias :math:`\\tau_{bias}` can be added
        
    .. math:: P_{f} = \{ f ~ | ~ J^Tf = NF + \\tau_{bias}, \quad F_{min} \leq F \leq F_{max}\}

    Based on the ``iterative_convex_hull_method`` algorihtm.

    Args:
        J: jacobian matrix
        N: moment arm matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tolerance: tolerance for the polytope calculation
        torque_bias: torque bias optional (gravity or movement or applied forces ....) 
        options: dictionary of options for the polytope calculation (currently only ``calculate_faces`` is supported)

    Returns
    ---------
        polytope(Polytope):
            polytope object with the following attributes ``vertices``, torque vertices ``torque_vertices``, muscle force vertices ``muscle_force_vertices``, half-plane representation ``H``, ``d``, (and ``face_indices`` and ``faces`` if option ``calculate_faces`` is set to ``True``)

    """
    f_vert, H, d, faces , F_vert, t_vert = iterative_convex_hull_method(J.T, N, F_min, F_max, tol, bias=torque_bias)
    
    # construct polytope object
    poly = Polytope(vertices=f_vert, H=H, d=d)
    poly.torque_vertices = t_vert
    poly.mucsle_force_vertices = F_vert
    poly.face_indices = faces
    # calculate faces if option is set to True
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] is True:
        poly.faces = face_index_to_vertex(poly.vertices, faces)
    return poly

def velocity_polytope(J, N, dl_min=None , dl_max=None, dq_max=None, dq_min=None, tol=1e-5, options=None):
    """
    A function calculating the polytopes of achievable velocity based 
    on the jacobian matrix `J` and moment arm matrix `N`

    If only muscle contraction velocity limits are given

    .. math:: P_{v,\dot{l}} = \{ \dot{x} ~ | ~ L\dot{q} = \dot{l},~~ J\dot{q} = \dot{x}, \quad \dot{l}_{min} \leq \dot{l} \leq \dot{l}_{max}\}

    If only joint velocity limits are given

    .. math:: P_{v,\dot{q}} = \{ \dot{x} ~ | ~ J\dot{q} = \dot{x}, \quad \dot{q}_{min} \leq \dot{q} \leq \dot{q}_{max}\}

    If both are provided thefunction will calculate both and intersect them (much longer execution time)

    .. math:: P_{v} = P_{v,\dot{l}}~~ \\bigcap ~~P_{v,\dot{q}}


    Based on the ``iterative_convex_hull_method`` algorihtm.

    Args:
        J: jacobian matrix
        N: moment arm matrix :math:`L = -N^T`
        dl_min: minimal achievable muscle contraction veclocity
        dl_max: maximal achievable muscle contraction veclocity
        dq_min: minimal achievable joint velocity
        dq_max: maximal achievable joint velocity
        tol: tolerance for the polytope calculation
        options: dictionary of options for the polytope calculation (currently only ``calculate_faces`` is supported)
        
    Returns
    ---------
        polytope(Polytope):
            polytope object with the following attributes ``vertices``, joint velocity vertices ``dq_vertices`` and muscle elongation vertices ``dl_vertices``, half-plane representation ``H``, ``d``, (and ``face_indices`` and ``faces`` if option ``calculate_faces`` is set to ``True``)
    """



    # initial value for inequality constraints
    G_in, h_in = None, None

    
    if dl_max is not None and dl_min is not None:
        v_vert, H, d, faces , dl_vert, dq_vert = iterative_convex_hull_method(A=-N.T, B=np.eye(dl_min.shape[0]), P = J, y_min=dl_min, y_max=dl_max, tol=tol)
        # construct polytope object
        poly = Polytope(vertices=v_vert, H=H, d=d)
        poly.dq_vertices = dq_vert
        poly.dl_vertices = dl_vert

    # if limits on joint velocity are given
    if dq_max is not None and dq_min is not None:
        dq_max = np.array(dq_max).reshape((-1,1))
        dq_min = np.array(dq_min).reshape((-1,1))
        poly_dq = robot.velocity_polytope(J, dq_max, dq_min,  options)
        if dl_max is not None:
            poly.vertices = None
            poly.H = np.vstack((H, poly_dq.H))
            poly.d = np.hstack((d.flatten(), poly_dq.d.flatten()))
            poly.find_vertices()
        else:
            poly = poly_dq
    

    # calculate faces if option is set to True
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] is True:
        poly.find_faces()

    return poly

def torque_to_muscle_force(N, F_min, F_max, tau, options="lp"):
    """
    A function calculating muscle forces needed to create the joint torques tau.
    It uses eaither the linear programming or quadratic programming, set with the ``options`` parameter.

    The quadratic programming problem is formulated as:

    .. math:: F = \\text{arg}\min_F ||F||^2 \quad s.t. \quad N^TF = \\tau, \quad F_{min} \leq F \leq F_{max}

    The linear programming problem is formulated as:
    
    .. math::  F = \\text{arg}\min_F \sum_i \\frac{1}{F_{max,i} - F_{min,i}}F_i \quad s.t. \quad N^TF = \\tau, \quad F_{min} \leq F \leq F_{max}

    Args:
        N: moment arm matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tau: joint torque
        options: dictionary of options (currently supported ``solver`` type to use: ``lp`` - linear programming (defualt), ``qp`` - quadratic programming)
        
    Returns:
        F(list): list of muscle forces
        
    """
    F_min = np.array(F_min)
    F_max = np.array(F_max)
    F_range = F_max - F_min

    L = len(F_min)
    if "lp" in options :
        solvers_opt={'tm_lim': 100000, 'msg_lev': 'GLP_MSG_OFF', 'it_lim':1000}
        c = matrix(np.ones(L)/F_range)
        A = matrix(N)
        b = np.array(tau).astype('float')
        b = matrix(b)
        G = matrix(np.vstack((np.eye(L),-np.eye(L))))
        h = matrix(np.hstack((F_max, list(-np.array(F_min)))))
        res = cvxopt.glpk.lp(c=c,G=G,h=h,A=A,b=b,options=solvers_opt)
        # print(tau)
        if res[1] != None:
            return list(res[1])
        else:
            print("None")
            return F_min
    else:
        cvxopt.solvers.options['show_progress'] = False
        P = matrix(np.eye(L)/F_range/F_range)
        q = matrix(np.zeros(L))
        A = matrix(N)
        b = np.array(tau).astype('float')
        b =  matrix(b)
        G = matrix(np.vstack((np.eye(L),-np.eye(L))))
        h = matrix(np.hstack((F_max,list(-np.array(F_min)))))
        res = cvxopt.solvers.qp(P=P,q=q,G=G,h=h,A=A,b=b)
        if res['x'] != None:
            return list(res['x'])
        else:
            print("None")
            return F_min
