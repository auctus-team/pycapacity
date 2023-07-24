from pycapacity.objects import *
import numpy as np


# write a test for constructing the ellipsoid object
def test_ellipsoid_construction():
    e = Ellipsoid(rotation=np.eye(3), radii=np.zeros((3,1)))
    assert e.radii.shape == (3,1)

# write a test for constructing the polytope object
def test_polytope_construction():
    p = Polytope(vertices=np.zeros((3,1)))
    assert p.vertices.shape == (3,1)

# write a test for polytope construction from point cloud
def test_polytope_from_point_cloud():
    points = np.round(np.random.rand(3,10),2)
    p = Polytope()
    p.find_from_point_cloud(points)
    print(points, p.vertices, np.isin(p.vertices, points))
    assert p.vertices.shape[1] <= points.shape[1] and np.all(p.H@points - p.d[:,None] < 0.001) and np.all(np.isin(np.round(p.vertices,2), points))

# write a test for polytope half-plane representation from a set of vertices
def test_polytope_from_halfplane():
    points = np.round(np.random.rand(3,10),2)
    p = Polytope()
    p.find_from_point_cloud(points)
    p.find_halfplanes()
    assert np.all(p.H@points - p.d[:,None] < 0.001)

# write a test for polytope face representation from a set of vertices
def test_polytope_from_face():
    points = np.round(np.random.rand(3,10),2)
    p = Polytope()
    p.find_from_point_cloud(points)
    p.find_faces()
    # check if all faces in the face representation are in the vertex representation
    assert np.all(np.isin(p.faces,p.vertices))

# write a test for polytope vertex representation from a set of half-planes
def test_polytope_from_halfplane_to_vertex():
    points = np.round(np.random.rand(3,10),2)
    p = Polytope()
    p.find_from_point_cloud(points)
    p.find_vertices()
    # check if all vertices in the vertex representation are in the half-plane representation
    assert np.all(np.isin(np.round(p.vertices,2),points))

# write a test for polytope face representation from a set of vertices
def test_polytope_from_vertex_to_face():
    points = np.round(np.random.rand(3,10),2)
    p = Polytope(vertices=points)
    p.find_faces()
    # check if all faces in the face representation are in the vertex representation
    assert np.all(np.isin(p.faces,p.vertices))