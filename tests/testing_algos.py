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
    # s.t. y in [y_min, y_max]
    algos.hyper_plane_shift_method(B, y_min, y_max)
    # Ax = y
    # s.t. y in [y_min, y_max]
    y_min = np.zeros(n)
    y_max = np.ones(n)
    algos.vertex_enumeration_auctus(A, y_min, y_max)

    assert True
