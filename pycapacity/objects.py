"""
Overview
---------

There two  genertic classes implemented in this module:

* `Polytope <#pycapacity\.objects\.Polytope>`_: A generic class representing a polytope with different representations (vertex, half-plane, face)
* `Ellipsoid <#pycapacity\.objects\.Ellipsoid>`_: A generic class representing an ellipsoid with a list of radii and rotation matrix
"""

from pycapacity.algorithms import *

class Polytope:
    """
    A generic class representing a polytope with different representations (vertex, half-plane, face)

    Vertex representation is a list of vertices of the polytope

    .. math::
        \mathcal{H}\!-\!rep = \{x \in \mathbb{R}^n | Hx \leq d \}

        
    Half-plane representation is a list of inequalities defining the polytope

    .. math::
        \mathcal{V}\!-\!rep = \{x_{v1},~ x_{v2},~ \ldots, ~x_{vn} \}

    Face representation is a list of triangles forming faces of the polytope, each triangle is represented as a list of tree vertices

    .. math::
        \mathcal{F}\!-\!rep = \{ [x_{v1},~ x_{v2}, ~x_{v2}],  ~ \ldots , ~[x_{vi},~ x_{vj}, ~x_{vk}], \ldots \}


    :ivar vertices: vertex representation of the polytope
    :ivar H: half-plane representation of the polytope Hx<d - matrix H
    :ivar d: half-plane representation of the polytope Hx<d- vector d
    :ivar faces: face representation of the polytope - faces are represented as a list of triangulated vertices
    :ivar face_indices: face representation of the polytope - faces are represented as a list of indices of vertices


    Polytope object implements the following operators:

    - ``+`` : minkowski sum of two polytopes
    - ``-`` : intersection of two polytopes

    Examples:
        >>> from pycapacity.objects import *
        >>> import numpy as np
        >>> # create a polytope object
        >>> p = Polytope(vertices=np.random.rand(3,10))
        >>> # find half-plane representation of the polytope
        >>> p.find_halfplanes()
        >>> # find face representation of the polytope
        >>> p.find_faces()
        >>> # find vertex representation of the polytope
        >>> p.find_vertices()
        >>> # create another polytope object
        >>> p1 = Polytope(vertices=np.random.rand(3,10))
        >>> # minkowski sum of two polytopes
        >>> p_sum = p + p1
        >>> # intersection of two polytopes
        >>> p_int = p & p1


    Additionally for robot's force polytopes will have additional attributes:
    
    - **torque_vertices**: (if applicable) joint torques corresponding to the vertices of the polytope

    Additionally for human's force polytopes will have additional attributes:

    - **torque_vertices**: (if applicable) joint torques corresponding to the vertices of the polytope
    - **muscle_forces_vertices**: (if applicable) muscle forces corresponding to the vertices of the polytope

    Human's velocity polytopes will have additional attributes:

    - **dq_vertices**: (if applicable) joint velocities corresponding to the vertices of the polytope
    - **dl_vert**: (if applicable) muscle elongation velocities corresponding to the vertices of the polytope
    """
    
    def __init__(self, vertices=None,faces=None,face_indices=None,H=None,d=None):
        """
        A constructor of the polytope object

        Args:
            vertices (np.array): vertices of the polytope (optional)
            faces (list): list of triangulated faces of the polytope (optional)
            face_indices (list): list of indices of vertices of the polytope (optional)
            H (np.array): half-plane representation of the polytope Hx<d - matrix H (optional)
            d (np.array): half-plane representation of the polytope Hx<d- vector d (optional)

        """
        self.vertices = vertices
        self.faces = faces
        self.face_indices = face_indices
        self.H = H
        self.d = d
    
    def find_vertices(self):
        """
        A function calculating the vertex representation of the polytope from the half-plane representation

        """
        if self.H is not None and self.d is not None:
            # finding vertices of the polytope from the half-plane representation
            self.vertices, self.face_indices = hspace_to_vertex(self.H,self.d)
        else:
            print("No half-plane representation of the polytope is available")

    def find_halfplanes(self):
        """
        A function calculating the half-plane representation of the polytope from the vertex representation
        """
        if self.vertices is not None:
            self.H, self.d = vertex_to_hspace(self.vertices)
        else:
            print("No vertex representation of the polytope is available")


    def find_faces(self):
        """
        A function calculating the face representation of the polytope from the vertex representation
        """
        if self.vertices is not None:
            if self.face_indices is not None:
                self.faces = face_index_to_vertex(self.vertices,self.face_indices)
            else:
                self.face_indices = vertex_to_faces(self.vertices)
                self.faces = face_index_to_vertex(self.vertices,self.face_indices)
        else:
            print("No vertex representation of the polytope is available")

    def find_from_point_cloud(self, points):
        """
        A function updating the polytope object from a point cloud it calculates the vertex and half-plane representation of the polytope. 
        
        Note:
            The polytope will be constructed as a convex hull of the point cloud

        Args:
            points (np.array): an array of points forming a point cloud 
        """
        self.H, self.d = vertex_to_hspace(points)
        self.vertices, self.face_indices = hspace_to_vertex(self.H,self.d)

    # minkowski sum of two polytopes
    def __add__(self, p):
        if self.vertices is None:
            self.find_vertices()
        if p.vertices is None:
            p.find_vertices()   
        vertices_sum = []
        for v1 in self.vertices.T:
            for v2 in p.vertices.T:
                vertices_sum.append(v1+v2)
        P_sum = Polytope()
        P_sum.find_from_point_cloud(points=np.array(vertices_sum).T)
        return P_sum
    
    # intersecting two polytopes
    def __and__(self, p):
        if self.H is None or self.d is None:
            self.find_halfplanes()
        if p.H is None or p.d is None:
            p.find_halfplanes()
        H_int = np.vstack((self.H,p.H))
        d_int = np.vstack((self.d,p.d))
        return Polytope(H=H_int,d=d_int)

    # implement chebyshev ball
    def chebyshev_ball(self):
        """
        Function calculating the Chebyshev ball of the polytope, the largest ball that can be inscribed in the polytope. 
        The function calculates the center and radius of the ball and returns an ellipsoid object representing the Chebyshev ball of the polytope

        Th function uses one Linear Programming problem to find the Chebyshev ball of the polytope.
        See also: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.HalfspaceIntersection.html#r9b902253b317-1

        Returns:
            Ellipsoid(Ellipsoid): an ellipsoid object representing the Chebyshev ball of the polytope
        
        """
        if self.H is None or self.d is None:
            self.find_halfplanes()
        center, radius = chebyshev_ball(self.H,self.d.reshape(-1,1))
        return Ellipsoid(radii=np.ones(self.H.shape[1],)*radius,rotation=np.eye(self.H.shape[1]), center=center)


class Ellipsoid:
    """
    A generic class representing an ellipsoid with a list of radii and rotation matrix, where 
    every column of the rotation matrix is a vector of the ellipsoid's axes

    .. math::
        \mathcal E = \{x ~|~ x^T R^T W R x \leq 1 \}

    where :math:`W = diag(r_1^{-2}, r_2^{-2}, \ldots, r_n^{-2})` and :math:`r_i` are radii of the ellipsoid, while :math:`R` is the rotation matrix
    

    :ivar center: ellipsoid center
    :ivar radii: radii of the ellipsoid
    :ivar rotation: rotation of the ellipsoid SO3 matrix
    """

    def __init__(self, radii=None, rotation=None, center=None):
        self.radii = radii
        self.rotation = rotation
        self.center = center