Polytope manipulation example: Inner sphere approximation
=============================================================

This example shows how to manipulate polytopes using the ``pycapacity`` package. 
In particular, it shows how to calculate the inner sphere approximation of a polytope, using the Chebyshev ball algorithm.

.. code-block:: python

    from pycapacity.objects import Polytope # import polytope object
    import numpy as np

    # this seed is used to generate the same image 
    # as in the examples in the docs 
    np.random.seed(12345)

    N = 10 # ten vertices
    m = 3   # space dimension

    points = np.array(np.random.rand(m,N))*2-1 # points

    # create a polytope object
    p = Polytope()
    p.find_from_point_cloud(points)

    # calculate the inner sphere approximation 
    s = p.chebyshev_ball()

    # plotting the polytope
    import matplotlib.pyplot as plt
    from pycapacity.visual import * # pycapacity visualisation tools

    fig = plt.figure(4)
    # draw polytopes
    plot_polytope(plot=fig, 
        polytope=p, 
        label='polytope', 
        face_color="blue", 
        edge_color='black',  
        alpha=0.2)
    plot_ellipsoid(plot=fig, ellipsoid=s, label='sphere')
    plt.legend()
    plt.show()

The output of the code is a visualisation of the the polytope and the inner sphere approximation.

.. image:: ../images/polyman_ball.png

