import numpy as np
import itertools
from scipy.spatial import ConvexHull, HalfspaceIntersection
from cvxopt import matrix
import cvxopt.glpk


def iterative_convex_hull_method(A, B, y_min, y_max, tol, P = None, bias = None,  G_in = None, h_in = None, G_eq = None, h_eq = None, max_iter=1000, verbose=False):
    """
    A function calculating the polytopes of achievable x for equations form:

    .. math:: z = By, \quad Ax = z
    .. math:: y_{min} \leq y \leq y_{max}
    
    or
    
    .. math:: Ax = By
    .. math:: y_{min} \leq y \leq y_{max}

    (optionally - additional projection matrix)

    .. math:: Az = By, \quad Pz = x
    .. math:: y_{min} \leq y \leq y_{max}

    (optionally - additional bias)

    .. math:: Az = By + bias
    .. math:: y_{min} \leq y \leq y_{max}

    (optionally - additional inequality constaints)

    .. math:: G_{in} y \leq h_{in}    
    
    (optionally - additional equality constaints)

    .. math:: G_{eq} y = h_{eq}

    Note:
        On-line feasible wrench polytope evaluation based on human musculoskeletal models: an iterative convex hull method
        A.Skuric,V.Padois,N.Rezzoug,D.Daney 

    Args:
        A: matrix
        B: matrix
        y_min: minimal values
        y_max: maximal values
        tol: tolerance for the polytope calculation
        P: an additional projection matrix 
        bias: bias in the intermediate space 
        G_in: matrix - inegality constraint G_in y < h_in
        h_in: vector - inegality constraint G_in y < h_in
        G_eq: matrix - equality constraint G_eq y = h_eq
        h_eq: vector - equality constraint G_eq y = h_eq
        max_iter: maximum number of iterations (number of linera programmes solved) - default 1000
        verbose: verbose program output - showing execution warnings - default False
    
    Returns
    ---------
        x_vert(list):  
            list of vertices
        H(list):  
            matrix of half-space representation `Hx<d`
        d(list): 
            vector of half-space representation `Hx<d`
        faces(list):   
            list of vertex indexes forming polytope faces  
        y_vert(list):
            list of y values producing x_vert
        z_vert(list):
            list of z values producing x_vert
        
    """
    # svd of jacobian
    n, m = A.shape
    nB,L = B.shape
    #L = len(y_min)
    if n != nB:
        raise ValueError('ICHM: Matrix A and B must have same number fo rows. In your case A: {}, B: {} '.format(A.shape,B.shape))

    if m > n or n > L :
        raise ValueError('ICHM: Matrix A and B dimensions must be colB >= rowB = rowA >= colA. In your case A: {}, B: {} '.format(A.shape,B.shape))

    if L!= len(y_max) or L!= len(y_min):
        raise ValueError('ICHM: Limits dimensios are not valid, should have {:d} entries. In your case y_min:{} and y_max:{}'.format(L,len(y_min),len(y_max)))


    y_min = np.array(y_min).reshape((-1,))
    y_max = np.array(y_max).reshape((-1,))

    if np.min(y_max - y_min) < 0:
        raise ValueError('ICHM: Limits not valid, minimal value is higher than maximal value.')

    u, s, v = np.linalg.svd(A.T)
    r = len(s)
    V1 = np.array(v.transpose()[:,:r])
    V2 = np.array(v.transpose()[:,r:])

    # if bias not defiend set it to zero
    if bias is None:
        bias = np.zeros((n,1))  
    else:
        bias = np.array(bias).reshape((n,1))  

    # optimisation setup
    if n > m:
        # for redundant case
        M = np.linalg.pinv(A).dot(B)
        x_bias = -np.linalg.pinv(A).dot(bias)
        # - intersection with the image of the J^T 
        Aeq = matrix(V2.T.dot(B)) 
        beq = matrix(V2.T.dot(bias))
        # for 1d jacobian case
        if m == 1:
            u = np.array([[1]])
    else:
        # for non redundant case 
        M = np.linalg.pinv(A).dot(B)
        x_bias = -np.linalg.pinv(A).dot(bias)
        # - no need to intersect with the image of J^T
        Aeq = None
        beq = None
        # for the case when A is an identity matrix
        if m == n and np.all(np.equal(A,np.identity(n))):
            # use u vector of the B instead of A
            u, sn, vn = np.linalg.svd(B)
        # for 1d jacobian case
        if m == 1:
            u = np.array([[1]])

    # if optional projection defined
    if P is not None:
        # check the size
        nP, mP = P.shape
        if mP != m or nP > mP:
            raise ValueError('ICHM: Matrix P dimensions error - (rows,cols) = P.shape should be: rows > cols and cols = {:d}. In your case P.shape = {}.'.format(m,P.shape))
        M = P.dot(M)
        x_bias = P.dot(x_bias)
        u = P.dot(u)  

    if G_in is not None:
        # check the size
        nG, mG = G_in.shape
        if mG != L:
            raise ValueError('ICHM: Matrix G_in dimensions error - (rows,cols) = G_in.shape should be: col = {:d}. In your case G_in.shape = {}.'.format(L,G_in.shape))
        if nG != len(h_in):
            raise ValueError('ICHM: Vector h_in dimensions error - should have {:d} entries. In your case h_in.shape = {}.'.format(nG,len(h_in)))    

        G = matrix(np.vstack((-np.identity(L),np.identity(L),G_in)))
        h = matrix(np.hstack((list(-np.array(y_min)),y_max, h_in)))
    else:
        G = matrix(np.vstack((-np.identity(L),np.identity(L))))
        h = matrix(np.hstack((list(-np.array(y_min)),y_max)))

    if G_eq is not None:
        # check the size
        nG, mG = G_eq.shape
        if mG != L: 
            raise ValueError('Matrix G_in dimensions error - (rows,cols) = G_in.shape should be: col = {:d}. In your case Geq.shape = {}.'.format(L,G_eq.shape))
        if nG != len(h_eq):
            raise ValueError('Vector h_in dimensions error - should have {:d} entries. In your case h_eq.shape = {}.'.format(nG,len(h_eq)))    
        if Aeq is not None:
            Aeq = matrix(np.vstack((Aeq,G_eq)))
            beq = matrix(np.hstack((beq,h_eq)))
        else:
            Aeq = matrix(G_eq)
            beq = matrix(h_eq)

    #solvers.options['show_progress'] = False
    solvers_opt = ""#"glpk"
    #solvers.options['glpk'] = dict(msg_lev='GLP_MSG_OFF')
    # solvers_opt.options['solver'] = 'mosek' # 'glpk'

    linprog_count = 0

    solvers_opt={'tm_lim': 100000, 'msg_lev': 'GLP_MSG_OFF', 'it_lim':10000}

    y_vert, x_p = [],[]
    for i in range(m):
        linprog_count = linprog_count + 2
        c = matrix((u[:,i].T).dot(M))
        res = cvxopt.glpk.lp(c=-c,  A=Aeq, b=beq, G=G,h=h, options=solvers_opt)
        if res[1] is not None:
            y_vert = stack(y_vert, res[1],'h')
        res = cvxopt.glpk.lp(c=c,  A=Aeq, b=beq, G=G,h=h, options=solvers_opt)
        if res[1] is not None:
            y_vert = stack(y_vert, res[1],'h')
    x_p  = M.dot(y_vert) + x_bias
    z_vert = B.dot(y_vert) + bias

    # if one directin only
    if m == 1:
        return x_p, np.array([[1],[-1]]), np.array([[np.max(x_p)],[-np.max(x_p)]]), [], y_vert, z_vert


    try:
        hull = ConvexHull(x_p.T, incremental=True)
    except:
        if(verbose): print("ICHM: Convex hull issue at init - continuing with a QJ option!")
        try:
            hull = ConvexHull(x_p.T, incremental=True, qhull_options="QJ")
        except:
            if(verbose): print("ICHM: Search stopped prematurely - inital convex hull not found!")
            z_vert = B.dot(y_vert) + bias
            x_vert  = M.dot(y_vert) + x_bias
            return x_vert, [], [], [], [], []

    # dictionary of face normals
    face_final = {}
    # set the initial max delta to some random value higher than the tolerance
    max_delta = tol*100 
    # iterate until the maxima
    # insteadl distance between the target and 
    # the aproximated polytope is under tol value
    while max_delta > tol and linprog_count <= max_iter:
        
        x_center = np.mean(x_p,axis=1)
        
        y_vert_new = []
        max_delta = 0
        for face, equation in zip(hull.simplices,hull.equations):
            
            # create a string index of the face 
            # this value is used as a hash map index
            # to store dyamically the faces that have been found as final
            face_key = str(np.sort(equation))
            # check if this face (face index) has been found as final
            if face_key in face_final.keys():
                continue;
            
            # update linprog counter
            linprog_count = linprog_count + 1

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
                
            res = np.array(res[1])

            # vertex distance from the face
            distance = np.abs( face_normal.dot( M.dot(res) + x_bias) -face_normal.dot( x_p[:,face[0]] ))

            if distance > tol:
                # new vertex found
                y_vert_new = stack(y_vert_new, res, 'h')
            else:
                face_final[face_key] = 1
            
            if distance > max_delta:
                max_delta = distance

        if len(y_vert_new):
            x_p_new = M.dot(y_vert_new) + x_bias

            x_p  = stack(x_p, x_p_new,'h')
            y_vert = stack(y_vert, y_vert_new,'h')

            z_new = B.dot(y_vert_new) + bias
            z_vert = stack(z_vert, z_new,'h')

            try:
                hull.add_points(x_p_new.T)
            except:
                if(verbose): print("ICHM: Convex hull issue - continuing with a QJ option!")
                hull = ConvexHull(x_p.T, incremental=True, qhull_options="QJ")
            
        elif max_delta > tol: 
            if(verbose): print("ICHM: Search stopped prematurely - search stuck at precision: {}!".format(max_delta))
            # raise error here instead
            break 

    if linprog_count >= max_iter:
        if(verbose): print("ICHM: Max iteration number reached: {}".format(max_iter))

    return hull.points.T, hull.equations[:,:-1], -hull.equations[:,-1], hull.simplices, y_vert, z_vert


def hyper_plane_shift_method(A, x_min, x_max, tol = 1e-15):
    """
    Hyper plane shifting method implementation used to solve problems of a form:

    .. math:: y = Ax
    .. math:: x_min <= x <= x_max


    Note:
        *Gouttefarde M., Krut S. (2010) Characterization of Parallel Manipulator Available Wrench Set Facets. In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht*


    This algorithm can be used to calcualte acceleration polytope, velocity polytoe and even 
    polytope of the joint achievable joint torques based on the muscle forces

    Args:
        A: projection matrix
        x_min: minimal values
        x_max: maximal values 
        
    Returns
    --------
        H(list):  
            matrix of half-space representation `Hx<d`
        d(list): 
            vector of half-space representation `Hx<d`
    """

    H = []
    d = []

    x_min = np.array(x_min).reshape((-1,1))
    x_max = np.array(x_max).reshape((-1,1))
    A = np.array(A)
    # Combination of n x n-1 columns of A
    #C = nchoosek(1:size(A,2),size(A,1)-1)
    C = np.array(list(itertools.combinations(range(A.shape[1]),A.shape[0]-1)))
    for comb in C:
        W = A[:,comb]
        U,S,V = np.linalg.svd(W.T)
        if ( A.shape[0] - np.linalg.matrix_rank(W) ) == 1 :
            c = V[-1,:]
            
            # Check for redundant constraint
            if len(H) :
                #diff = min(vecnorm(H - c,2,2))
                diff = np.min(np.linalg.norm(H-c,axis=1))
                if diff < tol:
                    c_exists = True
                else:
                    c_exists = False
            else: 
                c_exists = False

            # Compute offsets    
            if not c_exists :   
                I = c.dot(A)
                I_positive = np.where(I > 0)[0]
                I_negative = np.where(I < 0)[0]    
                d_positive = I[I_positive].dot(np.max([x_min[I_positive],x_max[I_positive]],0)) + I[I_negative].dot(np.min([x_min[I_negative],x_max[I_negative]],0))
                d_negative = -I[I_negative].dot(np.max([x_min[I_negative],x_max[I_negative]],0)) - I[I_positive].dot(np.min([x_min[I_positive],x_max[I_positive]],0))
                # Append constraints
                H = stack(H, c)
                H = stack(H, -c)
                d = stack(d, [d_positive, d_negative])

    return H, d

# maximal end effector force
def vertex_enumeration_auctus(A, b_max, b_min, b_bias = None):
    """
    Efficient vertex enumeration algorithm for a problem of a form:
    
    .. math:: Ax = b
    .. math:: b_{min} \leq b \leq b_{max}

    Optional (if b_bias added): 
    
    .. math:: Ax = b - b_{bias}
    .. math:: b_{min} \leq b \leq b_{max}

    Note:
        On-line force capability evaluation based on efficient polytope vertex search
        by A. Skuric, V. Padois, D. Daney

    Args:
        A:      system matrix A
        b_max:  maximal b 
        b_min:  minimal b  
        b_bias: b bias vector ( offset from 0 )

    Returns
    --------
        x_vertex(list): vertices of the polytope
        b_vartex(list): b values producing x_vertex

    """ 
    # Size calculation
    n, m = A.shape
    if m > n:
        raise ValueError('Matrix dimensions must be n >= m.  In your case n:'+str(n)+' m:'+str(m))

    if n!= len(b_max) or n!= len(b_min):
        raise ValueError('Limits dimensios are not valid, should have {:d} entries. In your case b_min:{} and b_max:{}'.format(n,len(b_min),len(b_max)))

    b_min = np.array(b_min).reshape(n,1)
    b_max = np.array(b_max).reshape(n,1)
    # if b_bias not specified
    if b_bias is None:
        b_bias = np.zeros((n,1))
    elif n!= len(b_bias) :
        raise ValueError('Bias dimensios are not valid, should have {:d} entries. In your case b_bias:{}'.format(n,len(b_bias)))
    else :
        b_bias = np.array(b_bias).reshape(n,1)

    # calculate svd
    U, S, V = np.linalg.svd(A.T)
    r = np.linalg.matrix_rank(A.T)
    V1 = np.array(V.transpose()[:,:m])
    V2 = np.array(V.transpose()[:,m:])

    # A matrix pseudo inverse
    A_inv = np.linalg.pinv(A)

    # matrix of axis vectors - for polytope search
    T_vec = np.diagflat(b_max-b_min)
    T_min = np.tile(b_min - b_bias,(1,2**m))
    b_max_bias = b_max - b_bias

    # all the face origin vector combiantions
    fixed_vectors_combinations = np.array(list(itertools.combinations(range(n),m)))
    permutations = np.array(list(itertools.product([0, 1], repeat=m))).T

    x_vertex = []
    b_vertex = []
    # A loop to go through all pairs of adjacent vertices
    for fixed_vectors in fixed_vectors_combinations:
        # find all n-m face vector indexes
        face_vectors = np.delete(range(n), fixed_vectors) 

        S = T_min.copy()
        for i in range(m): S[fixed_vectors[i], permutations[i] > 0] = b_max_bias[fixed_vectors[i]]
        S_v2 = V2.transpose().dot(S)

        # vectors to be used
        T = T_vec[:,face_vectors]
        # project the face vectors to the null space of the matrix A (using the V2)
        Z = V2.transpose().dot(-T)

        # figure out if some solutions can be discarded
        Z_min = Z.copy()
        Z_min[Z_min > 0] = 0
        Z_max = Z.copy()
        Z_max[Z_max < 0] = 0
        S_min = Z_min.dot(np.ones((n-m,1)))
        S_max = Z_max.dot(np.ones((n-m,1)))
        to_reject = np.any(S_v2 - S_min < - 10**-7, axis=0) + np.any(S_v2 - S_max > 10**-7, axis=0)
        if np.all(to_reject): # all should be discarded
            continue

        # remove the unfeasible solutions
        S = S[:,~to_reject]
        # project the fixed vectors to the null space of the matrix A (using the V2)
        S_v2 = V2.transpose().dot(S)
        
        # invert the matrix Z
        Z_inv = np.linalg.pinv(Z)
                                
        # calculate the solution of the x for each face
        X = Z_inv.dot(S_v2)
        # check if inverse correct - all error 0
        b_err = np.any( abs(S_v2 - Z.dot(X)) > 10**-7, axis=0) 
        # remove the solutions that are not in polytope 
        to_remove = (np.any(X < -10**-7, axis=0) + np.any(X - 1 > 10**-7 , axis=0)) + b_err
        X= X[:, ~to_remove]
        S= S[:, ~to_remove]
        if len(b_vertex) == 0:
            b_vertex =  S+T.dot(X)
        # add b vertex - corresponding to the x vertex
        b_vertex = np.hstack((b_vertex, S+T.dot(X)))
    
    # only unique vertices
    b_vertex = np.unique(np.around(b_vertex,7), axis=1)
    # calculate the forces based on the vertex torques
    x_vertex = A_inv.dot( b_vertex )
    return x_vertex, b_vertex

def chebyshev_center(A,b):
    """
    Calculating chebyshev center of a polytope given the half-space representation

    https://pageperso.lis-lab.fr/~francois.denis/IAAM1/scipy-html-1.0.0/generated/scipy.spatial.HalfspaceIntersection.html

    Args:
        A(list):  
            matrix of half-space representation `Ax<b`
        b(list): 
            vector of half-space representation `Ax<b`
    Returns:
        center(array): returns a chebyshev center of the polytope
    """
    # calculate the certices
    Ab_mat = np.hstack((np.array(A),-np.array(b)))

    # calculating chebyshev center
    norm_vector = np.reshape(np.linalg.norm(Ab_mat[:, :-1], axis=1), (A.shape[0], 1))
    c = np.zeros((Ab_mat.shape[1],))
    c[-1] = -1
    G = matrix(np.hstack((Ab_mat[:, :-1], norm_vector)))
    h = matrix(- Ab_mat[:, -1:])
    solvers_opt={'tm_lim': 100000, 'msg_lev': 'GLP_MSG_OFF', 'it_lim':10000}
    res = cvxopt.glpk.lp(c=c,  G=G, h=h, options=solvers_opt)
    return np.array(res[1][:-1]).reshape((-1,))

def hsapce_to_vertex(H,d):
    """
    From half-space representaiton to the vertex representation

    Args:
        H(list):  
            matrix of half-space representation `Hx<d`
        d(list): 
            vector of half-space representation `Hx<d`
    Returns
    --------
        vertices(list)  : vertices of the polytope
        face_indexes(list) : indexes of verteices forming triangulated faces of the polytope

    """
    if len(H):

        hd_mat = np.hstack((np.array(H),-np.array(d)))
        # calculate a feasible point inside the polytope
        feasible_point = chebyshev_center(H,d)
        # calculate the convex hull
        try:
            hd = HalfspaceIntersection(hd_mat,feasible_point)
            hull = ConvexHull(hd.intersections)
        except:
            print("H2V: Convex hull issue: using QJ option! ")
            try:
                hd = HalfspaceIntersection(hd_mat,feasible_point,qhull_options='QJ')
                hull = ConvexHull(hd.intersections)
            except:
                print("H2V: Convex hull issue: using Q0 option! ")
                hd = HalfspaceIntersection(hd_mat,feasible_point,qhull_options='Q0')
                hull = ConvexHull(hd.intersections)
        return hd.intersections.T, hull.simplices


def vertex_to_hspace(vertex):
    """
    Function transforming vertice to half-sapoce representation using a ConvexHull algorithm

    Args:
        vertex(array):  list of verteices

    Returns:
        H(list):  
            matrix of half-space representation `Hx<d`
        d(list): 
            vector of half-space representation `Hx<d`
    """
    if vertex.shape[0] == 1:
        faces = [0, 1]
    else:        
        hull = ConvexHull(vertex.T, qhull_options='QJ')
        faces = hull.simplices
    return  hull.equations[:,:-1], -hull.equations[:,-1]

def vertex_to_faces(vertex):
    """
    Function grouping the vertices to faces using a ConvexHull algorithm


    Args:
        vertex(array):  list of verteices

    Returns:
        faces(array) : list of triangle faces with vertex indexes which form them
    """
    if vertex.shape[0] == 1:
        faces = [0, 1]
    else:        
        hull = ConvexHull(vertex.T, qhull_options='QJ')
        faces = hull.simplices
    return faces

def order_index(points):
    """
    Order clockwise 2D points

    Args:
        points:  matrix of 2D points

    Returns
        indexes(array) : ordered indexes
    """
    px = np.array(points[0,:]).ravel()
    py = np.array(points[1,:]).ravel()
    p_mean = np.array(np.mean(points,axis=1)).ravel()

    angles = np.arctan2( (py-p_mean[1]), (px-p_mean[0]))
    sort_index = np.argsort(angles)
    return sort_index

def face_index_to_vertex(vertices, indexes):
    """
    Helping function for transforming the list of faces with indexes to the vertices

    Args:
        vertices: list of vertices
        indexes: list of vertex indexes forming faces

    Returns:
        faces: list of faces composed of vertices

    """
    dim = min(np.array(vertices).shape)
    if dim == 2:
        v = vertices[:,np.unique(indexes.flatten())]
        return v[:,order_index(v)]
    else:
        return [vertices[:,face] for face in indexes]

def stack(A, B, dir='v'):
    """
    Helping function enabling vertical and horisontal stacking of numpy arrays
    """
    if not len(A):
        return B
    elif not len(B):
        return A
    elif dir == 'v':
        return  np.vstack((A, B))
    else:
        return  np.hstack((A, B))
        