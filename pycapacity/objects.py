from pycapacity.algorithms import *

class Polytope:
    """
    A generic class representing a polytope with different representations (vertex, half-plane, face)


    :ivar vertices: vertex representation of the polytope
    :ivar H: half-plane representation of the polytope Hx<d - matrix H
    :ivar d: half-plane representation of the polytope Hx<d- vector d
    :ivar faces: face representation of the polytope - faces are represented as a list of triangulated vertices
    :ivar face_indices: face representation of the polytope - faces are represented as a list of indices of vertices


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
    
    def findVertices(self):
        """
        A function calculating the vertex representation of the polytope from the half-plane representation

        """
        if self.H is not None and self.d is not None:
            self.vertices, self.face_indices = hspace_to_vertex(self.H,self.d)
        else:
            print("No half-plane representation of the polytope is available")

    def findHalfplanes(self):
        """
        A function calculating the half-plane representation of the polytope from the vertex representation
        """
        if self.vertices is not None:
            self.H, self.d = vertex_to_hspace(self.vertices)
        else:
            print("No vertex representation of the polytope is available")


    def findFaces(self):
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


class Ellipsoid:
    """
    A generic class representing a polytope with different representations (vertex, half-plane, face)


    :ivar center: ellipsoid center
    :ivar radii: radii of the ellipsoid
    :ivar rotation: rotation of the ellipsoid SO3 matrix
    """

    def __init__(self, radii=None, rotation=None):
        self.radii = radii
        self.rotation = rotation