import numpy as np
import numpy.matlib
import itertools

# minkowski sum
from scipy.spatial import ConvexHull, HalfspaceIntersection

# import the algos
from pycapacity.algorithms import hyper_plane_shift_method, vertex_enumeration_auctus
from pycapacity.algorithms import order_index, face_index_to_vertex

# velocity manipulability calculation
def velocity_ellipsoid(J, dq_max):
    """
    velocity manipulability ellipsoid calculation

    Args:
        J: position jacobian
        dq_max:  maximal joint velocities
    Returns: 
        S(list):  list of axis lengths
        U(matrix): list of axis vectors
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
    acceleration ellipsoid calculation (dynamic manipulability ellipsoid)
   
    Returns:
     
    Args:
        J: matrix jacobian
        M: matrix inertia 
        t_max:  maximal joint torques
    Returns: 
        S(list):  list of axis lengths
        U(matrix): list of axis vectors
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
    force manipulability ellipsoid calculation

    Args:
        J: matrix jacobian
        t_max:  maximal joint torques
    Returns: 
        S(list):  list of axis lengths
        U(matrix): list of axis vectors
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


    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1
        t2_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2

    Returns:
        f_vertex(list):  vertices of the polytope
        t_vertex : joint torques corresponging to the force vertices
        t_bias : combined bias vector
    """
    # jacobian calculation
    Jac =  np.hstack((Jacobian1,Jacobian2))
    t_min = np.vstack((t1_min,t2_min))
    t_max = np.vstack((t1_max,t1_max))
    if t1_bias is None:
        t_bias = None
    else:
        t_bias = np.vstack((t1_bias, t2_bias))

    return force_polytope(Jac, t_max,t_min, t_bias)

def force_polytope_sum_withfaces(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias = None, t2_bias = None):
    """
    Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations.
    With ordered vertices into the faces.

    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1
        t2_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2

    Returns:
        f_vertex(list):  vertices of the polytope
        faces(list): list of vertex indexes belonging to faces
    """ 
    # calculate two polytopes
    f_vertex1, t_vertex1, t1_bias = force_polytope(Jacobian1, t1_max, t1_min, t1_bias)
    f_vertex2, t_vertex2, t2_bias = force_polytope(Jacobian2, t2_max, t2_min, t2_bias)
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

    Args:
        Jacobian:  position jacobian 
        t_max:  maximal joint torques 
        t_min:  minimal joint torques 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 

    Returns:
        f_vertex :  vertices of the polytope
        t_vertex : joint torques corresponging to the force vertices
        t_bias : bias vector used for the calculation
    """ 

    # jacobian calculation
    return vertex_enumeration_auctus(Jacobian.T, t_max, t_min, t_bias)
    
def force_polytope_withfaces(Jacobian, t_max, t_min, t_bias = None):
    """
    Force polytope representing the capacities of the two robots in a certain configuration.
    With vertices ordered into the faces

    Args:
        Jacobian:  position jacobian 
        t_max:  maximal joint torques 
        t_min:  minimal joint torques 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces

    Returns:
        f_vertex(list):  vertices of the polytope
        faces(list): list of vertex indexes belonging to faces
    """ 
    force_vertex, t_vertex, t_bias = force_polytope(Jacobian, t_max, t_min, t_bias)
    m, n = Jacobian.shape
    
    # find the polytope faces
    polytope_faces = []
    if force_vertex.shape[0] == 1:
        polytope_faces = [0, 1]
    else:        
        hull = ConvexHull(force_vertex.T)
        polytope_faces = hull.simplices
        
    return force_vertex, polytope_faces

def force_polytope_intersection_withfaces(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias=None, t2_bias=None):
    """
    Force polytope representing the intersection of the capacities of the two robots in certain configurations.
    With ordered vertices into the faces.

    Args:
        Jacobian1:  position jacobian robot 1
        Jacobian2: Jacobian2 position jacobian robot 2
        t_min1:  minimal joint torques robot 1
        t_min2:  minimal joint torques robot 2
        t_max1:  maximal joint torques robot 1
        t_max2:  maximal joint torques robot 2
        t1_bias:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1
        t2_bias:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2

    Returns:
        f_vertex(list):  vertices of the polytope
        faces(list): list of vertex indexes belonging to faces
    """
    force_vertex, t_vertex, t_bias = force_polytope_intersection(Jacobian1, Jacobian2, t1_max, t1_min, t2_max, t2_min, t1_bias, t2_bias)
    m, n = Jacobian1.shape
    t_max_int = np.vstack((t1_max,t2_max))
    t_min_int = np.vstack((t1_min,t2_min))

    polytope_faces = []
    if force_vertex.shape[0] == 1:
        polytope_faces = [0, 1]
    else:        
        hull = ConvexHull(force_vertex.T)
        polytope_faces = hull.simplices

    return force_vertex, polytope_faces


def velocity_polytope(Jacobian, dq_max, dq_min):
    """
    Velocity polytope calculating function

    Args:
        Jacobian:  position jacobian 
        dq_max:  maximal joint velocities 
        dq_min:  minimal joint velocities 

    Returns:
        velocity_vertex(list):  vertices of the polytope
    """ 
    velocity_vertex, H, d, vel_faces = hyper_plane_shift_method(Jacobian,dq_min,dq_max)
    return velocity_vertex

def velocity_polytope_withfaces(Jacobian, dq_max, dq_min):
    """
    Velocity polytope calculating function, with faces

    Args:
        Jacobian:  position jacobian 
        dq_max:  maximal joint velocities 
        dq_min:  minimal joint velocities 

    Returns:
        velocity_vertex(list):  vertices of the polytope
        faces(list): list of vertex indexes belonging to faces
    """ 
    velocity_vertex, H, d, vel_faces = hyper_plane_shift_method(Jacobian,dq_min,dq_max)
    return velocity_vertex, vel_faces

def acceleration_polytope(J, M, t_max, t_min, t_bias= None):
    """
    Acceleration polytope calculating function

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
    if t_bias is None:
        vertex, H, d, faces = hyper_plane_shift_method(B,t_min,t_max)
    else:
       vertex, H, d, faces = hyper_plane_shift_method(B,t_min-t_bias, t_max-t_bias)
    
    return vertex

def acceleration_polytope_withfaces(J, M, t_max, t_min, t_bias= None):
    """
    Acceleration polytope calculating function

    Args:
        J:  position jacobian 
        M:  inertia matrix 
        t_max:  maximal joint torque 
        t_min:  minimal joint torque 
        t_bias: bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces
    Returns:
        acceleration_vertex(list):  vertices of the polytope
        faces(list): list of vertex indexes belonging to faces
    """ 
    B = J.dot(np.linalg.pinv(M))
    if t_bias is None:
        vertex, H, d, faces_index = hyper_plane_shift_method(B,t_min,t_max)
    else:
        vertex, H, d, faces_index = hyper_plane_shift_method(B,t_min-t_bias,t_max-t_bias)
    return vertex, faces_index

# definition of the testing functions module
if __name__ == '__main__':
   
    n = 5 # nb joints
    m = 3 # cartesian forces
    J = np.random.rand(m,n)*2 - 1
    M = np.random.rand(n,n)
    t_min = np.zeros(n)
    t_max = np.ones(n)
    dq_min = -t_max
    dq_max = t_max

    force_polytope(J, t_min, t_max)
    force_polytope_withfaces(J, t_min, t_max)
    acceleration_polytope(J, M, t_min, t_max)
    acceleration_polytope_withfaces(J, M, t_min, t_max)
    velocity_polytope(J, dq_min, dq_max)
    velocity_polytope_withfaces(J, dq_min, dq_max)