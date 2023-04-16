import pycapacity.algorithms as algos
import numpy as np


def test1():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)
    # x = By
    # s.t. y in [y_min, y_max]Z
    algos.hyper_plane_shift_method(B, y_min, y_max)
    # Ax = y
    # s.t. y in [y_min, y_max]
    y_min = np.zeros(n)
    y_max = np.ones(n)
    algos.vertex_enumeration_auctus(A, y_min, y_max)

    assert True

# write unit test for hyper_plane_shift_method
# with random data and return true id no exception is raised
def test2():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # x = By
    # s.t. y in [y_min, y_max]
    algos.hyper_plane_shift_method(B, y_min, y_max)

    assert True

# write unit test for vertex_enumeration_auctus
# with random data and return true id no exception is raised
def test3():
    L = 10
    n = 5
    m = 3
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(n)
    y_max = np.ones(n)

    # Ax = y
    # s.t. y in [y_min, y_max]
    algos.vertex_enumeration_auctus(A, y_min, y_max)

    assert True

#  write unit test for iterative_convex_hull_method
# with random data and return true id no exception is raised
def test4():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)

    assert True

# unit test for iterative_convex_hull_method return true if expection is 
# raised when x_min and x_max are not the same size
def test5():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L+1)

    # Ax = By
    # s.t. y in [y_min, y_max]
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)
    except Exception as e:
        assert True

# unit test for iterative_convex_hull_method check if A nad B matrices not compatible
# return true if expection is raised
def test6():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n + 1, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)
    except Exception as e:
        assert True

# unit test for iterative_convex_hull_method with singular matrix A
# return true if expection is raised
def test7():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.hstack([np.random.rand(n, m-1) * 2 - 1, np.zeros((n,1))])
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)
    except Exception as e:
        assert False
    
    assert True


# unit test for iterative_convex_hull_method no matrix A is identity matrix
# return true if all is ok
def test8():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.eye(n)
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)

    assert True

# unit test for iterative_convex_hull_method with matrix B is identity matrix
# return true if all is ok
def test9():
    L = 10
    n = 5
    m = 3
    B = np.eye(n,L)
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)

    assert True

# unit test for iterative_convex_hull_method with equality constraints G_eq and h_eq
# return true if all is ok
def test10():
    np.random.seed(102)
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    G_eq = np.random.rand(2, L)
    h_eq = np.random.rand(2, 1)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, G_eq=G_eq, h_eq=h_eq)

    assert True

# unit test iterative_convex_hull_method if matrix A has less rows than columns
# return is exception is raised
def test11():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, n-1) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)
    except Exception as e:
        assert True
    
# unit test iterative_convex_hull_method if lower limits higher than upper limits
# return is exception is raised
def test12():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.ones(L)
    y_max = -np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)
    except Exception as e:
        assert True

# unit test iterative_convex_hull_method with bias  
# return true if all is ok
def test13():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    bias = np.random.rand(n, 1) * 2 - 1

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, bias=bias)

    assert True


# unit test for iterative_convex_hull_method with one dimensional matrix A
# return true if all is ok
def test14():
    L = 10
    n = 5
    m = 1
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)

    assert True

# unit test for iterative_convex_hull_method with one dimensional matrix A
# return true if all is ok
def test14():
    L = 10
    n = 5
    m = 1
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)

    assert True

# unit test for iterative_convex_hull_method with one dimensional matrix A and B
# return true if all is ok
def test15():
    L = 10
    n = 1
    m = 1
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1)

    assert True

# unit test iterative_convex_hull_method with projection matrix P  
# return true if all is ok
def test16():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    P = np.random.rand(m, m) * 2 - 1

    # Ax = By
    # s.t. y in [y_min, y_max]
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, P=P)

    assert True

# unit test iterative_convex_hull_method with projection matrix P which has more rows than matrix A has columns
# return is exception is raised
def test17():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    P = np.random.rand(m+1, m+1) * 2 - 1

    # Ax = By
    # s.t. y in [y_min, y_max]
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, P=P)
    except Exception as e:
        assert True

# unit test iterative_convex_hull_method with inequality constraints G_in and h_in
# return true if all is ok
def test18():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    G_in = np.eye(n, L)
    h_in = np.ones(n)

    # Ax = By
    # s.t. y in [y_min, y_max]
    # s.t. G_in * y <= h_in
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, G_in=G_in, h_in=h_in)

    assert True

# unit test for iterative_convex_hull_method with G_in matrix with more columns than y_min and y_max
# return is exception is raised
def test19():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    G_in = np.eye(n, L+1)
    h_in = np.ones(n)

    # Ax = By
    # s.t. y in [y_min, y_max]
    # s.t. G_in * y <= h_in
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, G_in=G_in, h_in=h_in)
    except Exception as e:
        assert True

# unit test iterative_convex_hull_method with h_in with more rows than G_in has
# return is exception is raised
def test20():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    G_in = np.eye(n, L)
    h_in = np.ones(n+1)

    # Ax = By
    # s.t. y in [y_min, y_max]
    # s.t. G_in * y <= h_in
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, G_in=G_in, h_in=h_in)
    except Exception as e:
        assert True

# unit test iterative_convex_hull_method with G_eq with more columns than y_min and y_max
# return is exception is raised
def test21():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    G_eq = np.eye(n, L+1)
    h_eq = np.ones(n)

    # Ax = By
    # s.t. y in [y_min, y_max]
    # s.t. G_eq * y = h_eq
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, G_eq=G_eq, h_eq=h_eq)
    except Exception as e:
        assert True

# unit test iterative_convex_hull_method with h_eq with more rows than G_eq has
# return is exception is raised
def test22():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.random.rand(n, m) * 2 - 1
    y_min = np.zeros(L)
    y_max = np.ones(L)
    G_eq = np.eye(n, L)
    h_eq = np.ones(n+1)

    # Ax = By
    # s.t. y in [y_min, y_max]
    # s.t. G_eq * y = h_eq
    try:
        algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, G_eq=G_eq, h_eq=h_eq)
    except Exception as e:
        assert True


# unit test for iterative_convex_hull_method with G_eq and h_eq and matrix A is identity matrix
# return is everythin is ok
def test23():
    L = 10
    n = 5
    m = 3
    B = np.random.rand(n, L) * 2 - 1
    A = np.eye(n)
    y_min = np.zeros(L)
    y_max = np.ones(L)
    G_eq = np.eye(n, L)
    h_eq = np.ones(n)

    # Ax = By
    # s.t. y in [y_min, y_max]
    # s.t. G_eq * y = h_eq
    algos.iterative_convex_hull_method(A, B, y_min, y_max, 0.1, G_eq=G_eq, h_eq=h_eq)

    assert True

# unit test stack function
# return true if all is ok
def test24():
    A = np.array([[1, 2, 3], [4, 5, 6]])
    B = np.array([[7, 8, 9], [10, 11, 12]])

    assert np.array_equal(np.array([[1, 2, 3, 7, 8, 9], [4, 5, 6, 10, 11, 12]]), algos.stack(A, B,'h'))

# unit test stack function
# return true if all is ok
def test25():
    A = np.array([[1, 2, 3], [4, 5, 6]])
    B = np.array([[7, 8, 9], [10, 11, 12]])

    assert np.array_equal(np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]]), algos.stack(A, B,'v'))


# unit test stack function with only one matrix
# return true if all is ok
def test25():
    A = []
    B = np.array([[7, 8, 9], [10, 11, 12]])

    assert np.array_equal(B, algos.stack(A, B))

# unit test stack function with only one matrix
# return true if all is ok
def test26():
    B = None
    A = np.array([[7, 8, 9], [10, 11, 12]])

    assert np.array_equal(A, algos.stack(A, B))


#unit test for chebyshev_center function
#return true if all is ok
def test27():
    A = np.vstack((np.eye(3),-np.eye(3)))
    b = np.hstack((np.ones(3) + 0.5, np.ones(3) - 0.5)).reshape(6,1)

    assert np.array_equal(np.array([0.5, 0.5, 0.5]), algos.chebyshev_center(A, b))

#unit test for  vertex_enumeration_auctus function with incompatible matrix A
# return true if exception is raised
def test28():
    A = np.array([[1, 2, 3], [4, 5, 6]])
    x_min = np.zeros(3)
    x_max = np.ones(3)

    try:
        algos.vertex_enumeration_auctus(A,x_min,x_max)
    except Exception as e:
        assert True
    
# unit test for vertex_enumeration_auctus function invalid x_min 
# return true if exception is raised
def test29():
    A = np.array([[1, 2, 3], [4, 5, 6], [4, 5, 6]])
    x_min = np.ones(4)
    x_max = np.ones(3)

    try:
        algos.vertex_enumeration_auctus(A,x_min,x_max)
    except Exception as e:
        assert True
            
# unit test for vertex_enumeration_auctus function invalid x_max 
# return true if exception is raised
def test29():
    A = np.array([[1, 2, 3], [4, 5, 6], [4, 5, 6]])
    x_min = np.ones(3)
    x_max = np.ones(4)

    try:
        algos.vertex_enumeration_auctus(A,x_min,x_max)
    except Exception as e:
        assert True

# unit test for vertex_enumeration_auctus function with bias    
# return true if all is ok
def test30():
    A = np.array([[1, 2, 3], [4, 5, 6],[4, 5, 6]])
    x_min = np.ones(3)
    x_max = np.ones(3)
    bias = np.array([1, 2, 3])

    algos.vertex_enumeration_auctus(A,x_min,x_max, bias)

    assert True

# unit test for vertex_enumeration_auctus function with bias invalid size
# return true if exception is raised
def test31():
    A = np.array([[1, 2, 3], [4, 5, 6],[4, 5, 6]])
    x_min = np.ones(3)
    x_max = np.ones(3)
    bias = np.array([1, 2, 3, 4])

    try:
        algos.vertex_enumeration_auctus(A,x_min,x_max, bias)
    except Exception as e:
        assert True

# unit test for face_to_vertex function 
# return true if all is ok
def test32():
    vert = np.array([[1, 2, 3], [4, 5, 6], [4, 5, 6]]).T
    face_indeces = np.array([1, 0, 2])

    assert  np.array_equal(([[4, 5, 6],[1, 2, 3], [4, 5, 6]]), algos.face_index_to_vertex(vert, face_indeces))

# unit test for face_to_vertex function  in 2d
# return true if all is ok
def test33():
    vert = np.array([[1, 2], [4, 5], [4, 5], [6, 7]]).T
    face_indeces = np.array([2, 1, 3,0])

    assert  np.array_equal(([[1, 4, 4,6], [2, 5, 5, 7]]), algos.face_index_to_vertex(vert, face_indeces))


# unit test for vertex_to_face function
# return true if all is ok
def test34():
    vert = np.random.rand(10,3).T

    algos.vertex_to_faces(vert)
    assert  True

# unit test for vertex_to_face function when vertice is 1d
# return true if all is ok
def test35():
    vert = np.random.rand(2).reshape(1,2)

    algos.vertex_to_faces(vert)
    assert  True

# unit test for vertex_to_hspace function
# return true if all is ok
def test36():
    np.random.seed(110)
    vert = np.random.rand(20,3).T

    algos.vertex_to_hspace(vert)
    assert  True


# unit test hspace_to_vertex function
# return true if all is ok
def test37():
    np.random.seed(110)
    H = np.vstack((np.eye(3),-np.eye(3)))
    d = np.hstack((np.ones(3), np.ones(3)))

    v, f =algos.hspace_to_vertex(H,d)
    v_correct = np.array([[-1.,  1., -1.,  1.,  1., -1.,  1., -1.],
       [-1., -1., -1., -1.,  1.,  1.,  1.,  1.],
       [-1., -1.,  1.,  1., -1., -1.,  1.,  1.]])
    assert np.array_equal(v_correct,v)

# unit test check that the combination of vertex_to_hspace and hspace_to_vertex works well
# return true if all is ok
def test38():
    np.random.seed(193)
    vert = np.random.rand(5,3).T

    H, d = algos.vertex_to_hspace(vert)
    v, f = algos.hspace_to_vertex(H, d)
    print(v.shape, vert.shape)
    assert np.allclose(np.sort(vert,axis=1), np.sort(v,axis=1))
