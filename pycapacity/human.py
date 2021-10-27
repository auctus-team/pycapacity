import numpy as np

from scipy.optimize import linprog 
from scipy.spatial import ConvexHull, HalfspaceIntersection

from cvxopt import matrix
import cvxopt.glpk

# hpsm
import itertools


def polytope_joint_torques(N, F_min, F_max, tol=1e-15):
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
    
def polytope_acceleration(J, N, M, F_min, F_max, tol=1e-15):
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

def polytope_force(J, N, F_min, F_max, tol):
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

def polytope_velocity(J, N, dl_min, dl_max, tol):
    """
    A function calculating the polytopes of achievable velocity based 
    on the jacobian matrix J and moment arm matrix N

    J.q = dl
    L.q = v
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
    return iterative_convex_hull_method(A=-N.T, B=np.eye(dl.shape[0]), P = J, y_mi=dl_min, y_max=dl_max, tol=tol)

def iterative_convex_hull_method(A, B, y_min, y_max, tol, P = None):
    """
    A function calculating the polytopes of achievable x for equations form:
    
    z = B.y
    A.x = z
    s.t. y_min <= y <= y_max
    
    or
    
    A.x = B.y
    s.t. y_min <= y <= y_max

    (optionally - additional projection matrix)
    A.z = B.y
    P.z = x
    s.t. y_min <= y <= y_max


    Args:
        A: matrix
        B: matrix
        y_min: minimal values
        y_max: maximal values
        tol: tolerance for the polytope calculation
        P: an additional projection matrix 
        
    Returns:
        x_vert(list):  list of cartesian force vertices
        H(list):  matrix of half-space representation Hx<d
        d(list):  vector of half-space representation Hx<d
        faces(list):   list of vertex indexes forming polytope faces  
    """
    # svd of jacobian
    m, n = A.T.shape
    L = len(y_min)

    u, s, v = np.linalg.svd(A.T)
    r = len(s)
    V1 = np.array(v.transpose()[:,:r])
    V2 = np.array(v.transpose()[:,r:])

    # optimisation setup
    if n > m:
        # for redundant case
        M = np.linalg.pinv(A).dot(B)
        # - intersection with the image of the J^T 
        Aeq = matrix(V2.T.dot(B)) 
        beq = matrix(np.zeros((n-m,1)))
        # for 1d jacobian case
        if m == 1:
            u = np.array([[1]])
    else:
        # for non redundant case 
        M = np.linalg.pinv(A).dot(B)
        # - no need to intersect with the image of J^T
        Aeq = None
        beq = None
        # for the case when A is an identity matrix
        if m == n and np.all(np.equal(A,np.identity(n))):
            # use u vector of the B instead of A
            u, sn, vn = np.linalg.svd(B)

    # if optional projection defined
    if P != None:
        M = P.dot(M)
        u = P.dot(u)

    G = matrix(np.vstack((-np.identity(L),np.identity(L))))
    h = matrix(np.hstack((list(-np.array(y_min)),y_max)))
    #solvers.options['show_progress'] = False
    solvers_opt = ""#"glpk"
    #solvers.options['glpk'] = dict(msg_lev='GLP_MSG_OFF')
    # solvers_opt.options['solver'] = 'mosek' # 'glpk'

    solvers_opt={'tm_lim': 100000, 'msg_lev': 'GLP_MSG_OFF', 'it_lim':10000}

    y_vert, x_p = [],[]
    for i in range(m):
        c = matrix((u[:,i].T).dot(M))
        res = cvxopt.glpk.lp(c=-c,  A=Aeq, b=beq, G=G,h=h, options=solvers_opt)
        y_vert = stack(y_vert, res[1],'h')
        res = cvxopt.glpk.lp(c=c,  A=Aeq, b=beq, G=G,h=h, options=solvers_opt)
        y_vert = stack(y_vert, res[1],'h')
    x_p  = M.dot(y_vert)

    try:
        hull = ConvexHull(x_p.T, incremental=True)
    except:
        try:
            hull = ConvexHull(x_p.T, incremental=True, qhull_options="QJ")
        except:
            z_vert = B.dot(y_vert)
            x_vert  = M.dot(y_vert)
            return x_vert, y_vert, z_vert, []

    n_faces, n_faces_old,  = len(hull.simplices), 0
    face_final, cnt = {}, 2*m

    while n_faces > n_faces_old:
        n_faces_old = n_faces
        
        x_center = np.mean(x_p)
        
        y_vert_new = []

        for face, equation in zip(hull.simplices,hull.equations):
            
            # create a string index of the face 
            # this value is used as a hash map index
            # to store dyamically the faces that have been found as final
            face_key = str(np.sort(face))
            # check if this face (face index) has been found as final
            if face_key in face_final.keys():
                continue; 

            # calculate the normal vector to the face
            face_normal = equation[:-1]

            # calculate the projection of the centroid on the normal of the face
            # to figure out the direction of the LP
            dir = np.mean(face_normal.dot(x_p[:,face[0]] - x_center)) > 0
            # use linear programming to find a vertex in the face_normal direciton
            c = matrix((face_normal).dot(M))
            if dir:
                res = cvxopt.glpk.lp(c=-c,  A=Aeq, b=beq, G=G,h=h, options=solvers_opt)
            else:
                res = cvxopt.glpk.lp(c=c,  A=Aeq, b=beq, G=G,h=h, options=solvers_opt)
                
            res = res[1]
            # a simple counter of linprog execuitions
            cnt = cnt+1

            # vertex distance from the face
            distance = np.abs( face_normal.dot( M.dot(list(res)) - x_p[:,face[0]] ) )
            if distance > tol:
                # new vertex found
                y_vert_new = stack(y_vert_new, np.array(res), 'h')
            else:
                face_final[face_key] = 1

        if len(y_vert_new):
            x_p_new = M.dot(y_vert_new)
            hull.add_points(x_p_new.T)
            y_vert = stack(y_vert, y_vert_new,'h')
            x_p  = stack(x_p, x_p_new,'h')
            
        n_faces = len(hull.simplices)

    z_vert = B.dot(y_vert)
    x_vert  = M.dot(y_vert)

    return x_vert, hull.equations[:,:-1], hull.equations[:,-1], hull.simplices

def hyper_plane_shift_method(A, x_min, x_max, tol = 1e-15):
    """
    Hyper plane shifting method implementation used to solve problems of a form:
    y = Ax
    s.t. x_min <= x <= x_max

    Hyperplane shifting method: 
    *Gouttefarde M., Krut S. (2010) Characterization of Parallel Manipulator Available Wrench Set Facets. In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht*


    This algorithm can be used to calcualte acceleration polytope, velocity polytoe and even 
    polytope of the joint achievable joint torques based on the muscle forces

    Args:
        A: projection matrix
        x_min: minimal values
        x_max: maximal values 
        
    Returns:
        H: half space representation matrix H - Hx < d
        d: half space representaiton vector d - Hx < d
        vertices: vertex representation of the polytope
    """
    H = []
    d = []

    x_min = np.array(x_min)
    x_max = np.array(x_max)
    # Combination of n x n-1 columns of A
    #C = nchoosek(1:size(A,2),size(A,1)-1)
    C = np.array(list(itertools.combinations(range(A.shape[1]),A.shape[0]-1)))
    for comb in range(C.shape[0]):
        W = A[:,C[comb,:]]
        U,S,V = np.linalg.svd(W.T)
        if ( A.shape[0] - len(S) ) == 1 :
            c = V[:,-1].T

            # Check for redundant constraint
            if len(H) :
                #diff = min(vecnorm(H - c,2,2))
                diff = np.min( np.sqrt( np.sum( np.power((H-c), 2), 1)))
                if diff < tol and diff > -tol :
                    c_exists = True
                else:
                    c_exists = False
            else: 
                c_exists = False

            # Compute offsets    
            if ~c_exists :   
                I = c.dot(A)          
                I_positive = np.where(I > 0)[0]
                I_negative = np.where(I < 0)[0]    
                d_positive = np.sum(I[I_positive] * np.max([x_min[I_positive],x_max[I_positive]],0)) + np.sum(I[I_negative] * np.min([x_min[I_negative],x_max[I_negative]],0))
                d_negative = -np.sum(I[I_negative] * np.max([x_min[I_negative],x_max[I_negative]],0)) - np.sum(I[I_positive] * np.min([x_min[I_positive],x_max[I_positive]],0))

                # Append constraints
                H = stack(H, c)
                H = stack(H, -c)
                d = stack(d, [[d_positive], [d_negative]])

    if len(H):
        # calculate the certices
        hd_mat = np.hstack((np.array(H),-np.array(d)))
        hd = HalfspaceIntersection(hd_mat,np.zeros(A.shape[0]))
        hull = ConvexHull(hd.intersections)
        return hd.intersections.T, H, d, hull.simplices
    else:
        return [], H, d, []

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

def stack(A, B, dir='v'):
    if not len(A):
        return B
    elif not len(B):
        return A
    elif dir == 'v':
        return  np.vstack((A, B))
    else:
        return  np.hstack((A, B))

if __name__ == "__main__":
  
    L = 20 # nb muslces
    n = 5 # nb joints
    m = 3 # cartesian forces
    N = (np.random.rand(n,L)*2 -1)
    J = np.random.rand(m,n)
    M = np.random.rand(n,n)
    F_min = np.zeros(L)
    F_max = np.ones(L)

    f, *junk = polytope_force(J,N, F_min, F_max, 0.1)
    a, *junk = polytope_acceleration(J, N, M, F_min, F_max)
    # direct acceleration polytope calculation
    a1, *junk = iterative_convex_hull_method(np.identity(m), J.dot(np.linalg.inv(M).dot(N)), F_min, F_max,0.1)
    v, *junk = polytope_acceleration(J, N, M, F_min, F_max)
