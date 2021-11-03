import numpy as np
from cvxopt import matrix
import cvxopt.glpk

# import the algos
from pycapacity.algorithms import iterative_convex_hull_method, hyper_plane_shift_method
from pycapacity.algorithms import stack, face_index_to_vertex


def joint_torques_polytope(N, F_min, F_max, tol=1e-15):
    """
    A function calculating the polytopes of achievable joint torques
    based on the moment arm matrix N :
    
    t = N.F
    st F_min <= F <= F_max

    Args:
        N: moment arm matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tolerance: tolerance for the polytope calculation
        
    Returns:
        t_vert(array):  list of torque vertices
        H(array):  half-space rep matrix H - H.t < d
        d(array):  half-space rep vector d
        faces: indexes of verteices forming the polytope faces
    """
    return hyper_plane_shift_method(N, F_min, F_max)
    
def acceleration_polytope(J, N, M, F_min, F_max, tol=1e-15):
    """
    A function calculating the polytopes of achievable accelerations
    based on the jacobian matrix J, moment arm matrix N and mass matrix M

    a = ddx = J.M^(-1).N.F
    st F_min <= F <= F_max

    Args:
        J: jacobian matrix
        N: moment arm matrix
        M: mass matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tolerance: tolerance for the polytope calculation
        
    Returns:
        a_vert(array):  list of acceleraiton vertices
        H(array):  half-space rep matrix H - H.a < d
        d(array):  half-space rep vectors d
        faces: indexes of verteices forming the polytope faces
    """
    return hyper_plane_shift_method(J.dot(np.linalg.inv(M).dot(N)),F_min,F_max)

def force_polytope(J, N, F_min, F_max, tol):
    """
    A function calculating the polytopes of achievable foreces based 
    on the jacobian matrix J and moment arm matrix N

    J^T.f = N.F
    st F_min <= F <= F_max

    Args:
        J: jacobian matrix
        N: moment arm matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tolerance: tolerance for the polytope calculation
        
    Returns:
        f_vert(list):  list of cartesian force vertices
        F_vert(list):  list of muscle force vertiecs
        t_vert(list):  list of joint torque vertices
        faces(list):   list of vertex indexes forming polytope faces  
    """
    return iterative_convex_hull_method(J.T, N, F_min, F_max, tol)

def velocity_polytope(J, N, dl_min , dl_max, tol):
    """
    A function calculating the polytopes of achievable velocity based 
    on the jacobian matrix J and moment arm matrix N

    L.q = dl
    J.q = v
    st dl_min <= dl <= dl_max

    Args:
        J: jacobian matrix
        N: moment arm matrix L = -N^T
        dl_min: minimal achievable muscle contraction veclocity
        dl_max: maximal achievable muscle contraction veclocity
        tolerance: tolerance for the polytope calculation
        
    Returns:
        v_vert(list):  list of cartesian velocity vertices
        dl_vert(list): list of muscle contraction velocity vertiecs
        q_vert(list):  list of joint angular velocity vertices
        faces(list):   list of vertex indexes forming velocity polytope faces  
    """
    return iterative_convex_hull_method(A=-N.T, B=np.eye(dl_min.shape[0]), P = J, y_min=dl_min, y_max=dl_max, tol=tol)

def torque_to_muscle_force(N, F_min, F_max, tau, options="lp"):
    """
    A function calculating muscle forces needed to create the joint torques tau

    Args:
        N: moment arm matrix
        F_min: minimal muscular forces (passive forces or 0)
        F_max: maximal isometric forces 
        tau: joint torque
        options: solver type to use: 'lp' - linear programming (defualt), 'qp' - quadratic programming
        
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


if __name__ == "__main__":
  
    L = 20 # nb muslces
    n = 5 # nb joints
    m = 3 # cartesian forces
    N = (np.random.rand(n,L)*2 -1)
    J = np.random.rand(m,n)
    M = np.random.rand(n,n)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    force_polytope(J,N, F_min, F_max, 0.1)
    acceleration_polytope(J, N, M, F_min, F_max)
    # direct acceleration polytope calculation
    iterative_convex_hull_method(np.identity(m), J.dot(np.linalg.inv(M).dot(N)), F_min, F_max,0.1)
    acceleration_polytope(J, N, M, F_min, F_max)
