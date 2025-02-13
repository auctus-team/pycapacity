r"""
Overview
---------

This is a python module which implements different human performance metrics based on their musculoskeletal models


* acceleration `polytope <#pycapacity\.human\.acceleration_polytope>`_ and `ellipsoid <#pycapacity\.human\.acceleration_ellipsoid>`_
* velocity (manipulability) `polytope <#pycapacity\.human\.velocity_polytope>`_ and `ellipsoid <#pycapacity\.human\.velocity_ellipsoid>`_
* force `polytope <#pycapacity\.human\.force_polytope>`_ and `ellipsoid <#pycapacity\.human\.force_ellipsoid>`_
* `joint torque polytope <#pycapacity\.human\.joint_torques_polytope>`_

"""

import numpy as np
from cvxopt import matrix
import cvxopt.glpk

# import the algos
from pycapacity.algorithms import *
from pycapacity.objects import *
import pycapacity.robot as robot


def velocity_ellipsoid(J, N, dl_max):
    r"""
    Human musculoskeletal velocity ellipsoid calculation

    .. math:: E_f = \{\dot{x}~ |~ J\dot{q} = \dot{x},~ L\dot{q} = \dot{l} \quad ||W^{-1}\dot{l}|| \leq 1\}
   
    where

    .. math:: W=diag(\dot{l}_{max})
    
    Args:
        J: position jacobian
        N: moment arm matrix (:math:`N = -L^T`, where :math:`L` is the muscle length jacobian)
        dl_max:  maximal joint velocities 

    Returns
    ---------
        ellipsoid(Ellipsoid):
            ellipsoid object with the following attributes ``radii``, ``axes``
    """ 
    # jacobian calculation
    Jac = J@np.linalg.pinv(-N.T)
    # limits scaling
    W = np.diagflat(dl_max)
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))

    # create the ellipsoid from the singular values and the unit vector angle
    ellipsoid = Ellipsoid(radii=S, rotation=U)
    return ellipsoid

def acceleration_ellipsoid(J, M, N, F_max):
    r"""
    Human musculoskeletal acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
   
    .. math:: E_a = \{\ddot{x}~ |~ \ddot{x} = JM^{-1}NF, \quad ||W^{-1}F|| \leq 1\}

    where

    .. math:: W=diag(F_{max})

    Args:
        J: matrix jacobian
        M: matrix inertia 
        N: moment arm matrix
        F_max:  maximal muscular forces
        
    Returns
    ---------
        ellipsoid(Ellipsoid):
            ellipsoid object with the following attributes ``radii``, ``axes``
    """ 
    # jacobian calculation
    Jac = J@np.linalg.pinv(M)@N
    # limits scaling
    W = np.linalg.pinv(np.diagflat(F_max))
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # create the ellipsoid from the singular values and the unit vector angle
    ellipsoid = Ellipsoid(radii=S, rotation=U)
    return ellipsoid

def force_ellipsoid(J, N, F_max):
    r"""
    Human musculoskeletal force ellipsoid calculation

    .. math:: E_f = \{f~ |~ NF  = J^Tf,\quad ||W^{-1}F|| \leq 1\}

    where

    .. math:: W=diag(F_{max})
        
    Args:
        J: matrix jacobian
        N: moment arm matrix
        F_max:  maximal muscular forces

    Returns
    ---------
        ellipsoid(Ellipsoid):
            ellipsoid object with the following attributes ``radii``, ``axes``
    """ 
    # jacobian calculation
    Jac = J@np.linalg.pinv(N.T)
    # limits scaling
    W = np.linalg.pinv(np.diagflat(F_max))
    # calculate the singular value decomposition
    U, S, V = np.linalg.svd(Jac.dot(W))
    # create the ellipsoid from the singular values and the unit vector angle
    ellipsoid = Ellipsoid(radii=np.divide(1,S), rotation=U  )
    return ellipsoid


def joint_torques_polytope(N, F_min, F_max, tol=1e-5, options=None):
    r"""
    A function calculating the polytopes of achievable joint torques
    based on the moment arm matrix `N` :
    
    .. math:: P_{t} = \{ \\tau ~ | ~ \\tau= NF, \quad F_{min} \leq F \leq F_{max}\}

    Based on the ``hyper_plane_shifting_method`` algorithm.

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
    r"""
    A function calculating the polytopes of achievable accelerations
    based on the jacobian matrix `J`, moment arm matrix `N` and mass matrix `M`

    .. math:: P_{a} = \{ \ddot{x} ~ | ~ \ddot{x} = JM^{-1}NF, \quad F_{min} \leq F \leq F_{max}\}

    Based on the ``hyper_plane_shifting_method`` algorithm (default) or the ``iterative_convex_hull_method`` algorithm (options['algorithm'] = 'ichm').

    Args:
        J: jacobian matrix
        N: moment arm matrix
        M: mass matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tolerance: tolerance for the polytope calculation
        options: dictionary of options for the polytope calculation (currently supported options are ``calculate_faces`` and ``algorithm``)
        
    Returns
    ---------
        poly(Polytope):
            polytope object with the following attributes ``vertices``, half-plane representation ``H``, ``d``, (and ``face_indices`` and ``faces`` if option ``calculate_faces`` is set to ``True``)
    """

    if options is not None and 'algorithm' in options.keys() and options['algorithm'] == 'ichm':
        vert, H, d, faces , F_vert, t_vert = iterative_convex_hull_method(
            A = np.eye(J.shape[0]),
            B = J.dot(np.linalg.inv(M).dot(N)), 
            y_min=F_min, 
            y_max= F_max, 
            tol=tol)
        # construct polytope object
        poly = Polytope(vertices=vert, H=H, d=d)
        poly.face_indices = faces
    else:
        H,d = hyper_plane_shift_method(J.dot(np.linalg.inv(M).dot(N)),F_min,F_max)
        vert, faces = hspace_to_vertex(H,d) 
        # construct polytope object
        poly = Polytope(vertices=vert, H=H, d=d)
        poly.face_indices = faces

   
    # calculate faces if requested
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] is True:
        poly.faces = face_index_to_vertex(poly.vertices, faces)
    return poly

def force_polytope(J, N, F_min, F_max, tol, torque_bias=None, options=None):
    r"""
    A function calculating the polytopes of achievable forces based 
    on the jacobian matrix `J` and moment arm matrix `N`

    .. math:: P_{f} = \{ f ~ | ~ J^Tf = NF, \quad F_{min} \leq F \leq F_{max}\}

    optionally an additional bias :math:`\\tau_{bias}` can be added
        
    .. math:: P_{f} = \{ f ~ | ~ J^Tf = NF + \\tau_{bias}, \quad F_{min} \leq F \leq F_{max}\}

    Based on the ``iterative_convex_hull_method`` algorithm.

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
    poly.muscle_force_vertices = F_vert
    poly.face_indices = faces
    # calculate faces if option is set to True
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] is True:
        poly.faces = face_index_to_vertex(poly.vertices, faces)
    return poly


def velocity_polytope(J, N=None, dl_min=None , dl_max=None, dq_max=None, dq_min=None, tol=1e-5, options=None):
    r"""
    A function calculating the polytopes of achievable velocity based 
    on the jacobian matrix `J` and moment arm matrix `N`

    If only muscle contraction velocity limits are given

    .. math:: P_{v,\dot{l}} = \{ \dot{x} ~ | ~ L\dot{q} = \dot{l},~~ J\dot{q} = \dot{x}, \quad \dot{l}_{min} \leq \dot{l} \leq \dot{l}_{max}\}

    If only joint velocity limits are given

    .. math:: P_{v,\dot{q}} = \{ \dot{x} ~ | ~ J\dot{q} = \dot{x}, \quad \dot{q}_{min} \leq \dot{q} \leq \dot{q}_{max}\}

    If both are provided the function will calculate

    .. math:: P_{v,\dot{l}} = \{ \dot{x} ~ | ~ J\dot{q} = \dot{x}, \quad \dot{q}_{min} \leq \dot{q} \leq \dot{q}_{max}, ~~  \dot{l}_{max} \leq L\dot{q} \leq \dot{l}_{max}\}


    Based on the ``iterative_convex_hull_method`` algorithm.

    Args:
        J: jacobian matrix
        N: moment arm matrix :math:`L = -N^T`
        dl_min: minimal achievable muscle contraction velocity
        dl_max: maximal achievable muscle contraction velocity
        dq_min: minimal achievable joint velocity
        dq_max: maximal achievable joint velocity
        tol: tolerance for the polytope calculation
        options: dictionary of options for the polytope calculation (currently only ``calculate_faces`` is supported)
        
    Returns
    ---------
        polytope(Polytope):
            polytope object with the following attributes ``vertices``, joint velocity vertices ``dq_vertices`` and muscle elongation vertices ``dl_vertices``, half-plane representation ``H``, ``d``, (and ``face_indices`` and ``faces`` if option ``calculate_faces`` is set to ``True``)
    """


    # if limits on joint velocity are given
    if dq_max is not None and dq_min is not None:
        dq_max = np.array(dq_max).reshape((-1,1))
        dq_min = np.array(dq_min).reshape((-1,1))

        # if limits both limits are given
        if dl_max is not None and dl_min is not None:
            # caluclate the polytope of achievable velocities
            # given joint velocity limits 
            # and muscle contraction velocity limits transformed to joint velocity limits
            v_vert, H, d, faces , dq_vert, dv_vert = iterative_convex_hull_method(
                A=np.eye(J.shape[0]), 
                B=J, 
                y_min=dq_min, 
                y_max=dq_max,
                G_in = np.vstack((-N.T, N.T)),
                h_in = np.hstack((dl_max, -dl_min)),
                tol=tol
                )
            # construct polytope object
            poly = Polytope(vertices=v_vert, H=H, d=d)
            poly.dq_vertices = dq_vert
            poly.dl_vertices = -N.T@dq_vert
        else:
            # if only joint velocity limits are given
            # caluclate the polytope of achievable velocities
            # the same as the robot velocity polytope
            poly = robot.velocity_polytope(J, dq_max, dq_min,  options)
    
    elif dl_max is not None and dl_min is not None:
        # if only muscle contraction velocity limits are given
        # caluclate the polytope of achievable velocities
        # given muscle contraction velocity limits only
        v_vert, H, d, faces , dl_vert, dq_vert = iterative_convex_hull_method(
            A=-N.T, 
            B=np.eye(dl_min.shape[0]), 
            P = J, 
            y_min=dl_min, 
            y_max=dl_max, 
            tol=tol
            )
        # construct polytope object
        poly = Polytope(vertices=v_vert, H=H, d=d)
        poly.dq_vertices = dq_vert
        poly.dl_vertices = dl_vert
    

    # calculate faces if option is set to True
    if options is not None and 'calculate_faces' in options.keys() and options['calculate_faces'] is True:
        poly.find_faces()

    return poly

def torque_to_muscle_force(N, F_min, F_max, tau, options="lp"):
    r"""
    A function calculating muscle forces needed to create the joint torques tau.
    It uses either the linear programming or quadratic programming, set with the ``options`` parameter.

    The quadratic programming problem is formulated as:

    .. math:: F = \\text{arg}\min_F ||F||^2 \quad s.t. \quad N^TF = \\tau, \quad F_{min} \leq F \leq F_{max}

    The linear programming problem is formulated as:
    
    .. math::  F = \\text{arg}\min_F \sum_i \\frac{1}{F_{max,i} - F_{min,i}}F_i \quad s.t. \quad N^TF = \\tau, \quad F_{min} \leq F \leq F_{max}

    Args:
        N: moment arm matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tau: joint torque
        options: dictionary of options (currently supported ``solver`` type to use: ``lp`` - linear programming (default), ``qp`` - quadratic programming)
        
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
