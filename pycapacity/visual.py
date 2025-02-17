"""
Overview
---------

This is a python module helping to visualise 2d and 3d polytopes and ellipsoids. It is based on the module  ``matplotlib``.

* visualising 2d and 3d `polytope <#pycapacity\.visual\.plot_polytope>`_
* visualising 2d and 3d polytope `faces <#pycapacity\.visual\.plot_polytope_faces>`_ and `vertices  <#pycapacity\.visual\.plot_polytope_vertex>`_
* visualising 2d and 3d `ellipsoids <#pycapacity\.visual\.plot_ellipsoid>`_

"""

import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import mpl_toolkits
from matplotlib.patches import Ellipse
import numpy as np
from pycapacity.objects import *

def plot_polytope(polytope, plot=None, color=None, vertex_color='black', face_color=None, edge_color=None, alpha=None,label=None, center=None, scale=1.0, show_vertices=True, wireframe=False):
    """
    A polytope plotting function in 2d and 3d. It plots the polytope faces and vertices from the polytope object.

    Note:
        ``plot`` parameter can be either ``matplotlib.pyplot`` or ``matplotlib.axes.Axes`` or ``matplotlib.figure.Figure`` or ``mpl_toolkits.mplot3d.axes3d.Axes3D``.

    Args:
        plot:  matplotlib ax to plot on, can be either ``matplotlib.pyplot`` or ``matplotlib.axes.Axes`` or ``matplotlib.figure.Figure`` or ``mpl_toolkits.mplot3d.axes3d.Axes3D``
        polytope(Polytope) :  polytope object it has to be provided
        face_color:  polytope face color (optional)
        edge_color:  polytope edge color  (optional)
        vertex_color:  polytope vertex color  (optional)
        alpha:  polytope opacity  (optional)
        label:  legend label (optional)
        center: offset the polytope (optional)
        scale: scale the polytope with a scalar (optional)
        show_vertices: show the vertices of the polytope (optional)
        wireframe: show the polytope as a wireframe (optional)
        color:  polytope color - it can be either a string (one color for faces, edges and vertices) or a list of 3 values for face, edge and vertex color (optional)

    """
    if not isinstance(polytope, Polytope):
        print("no polytope provided")
        return plot
    
    # if color is one value, use it for all
    if color is not None and (isinstance(color, str) or len(color) == 1):
        face_color=color, 
        edge_color=color, 
        vertex_color=color
    # if color is a list of 3 values, use it for face, edge and vertex
    elif color is not None and len(color) == 3:
        face_color, edge_color, vertex_color = color

    if vertex_color is None:
        show_vertices = False

    if label is None:
        label = ''
    if show_vertices:
        if polytope.vertices is None:
            print("Visual: no vertices found, calculating vertices")
            polytope.find_vertices()
        if polytope.vertices is not None:
            plot = plot_polytope_vertex(polytope=polytope, plot=plot, label=label+' vertex', color=vertex_color, center=center, scale=scale)
        else:
            print("Visual: cannot find vertices")
           
    if polytope.faces is None:
        print("Visual: no faces found, calculating faces")
        polytope.find_faces()
    if polytope.faces is not None:
        plot = plot_polytope_faces(polytope=polytope, plot=plot, face_color=face_color, edge_color=edge_color, alpha=alpha,label=label+' faces', center=center, scale=scale, wireframe=wireframe)
    else:
        print("Visual: cannot find faces")

    return plot

def plot_polytope_faces(faces=None, polytope=None, plot=None, face_color=None, edge_color=None, alpha=None,label=None, center=None, scale=1.0, wireframe=False):
    """ 
    Polytope faces plotting function in 2d and 3d. 
    
    Note:
        ``plot`` parameter can be either matplotlib.pyplot or matplotlib.axes.Axes or matplotlib.figure.Figure.

    Examples:
        >>> import pycapacity.visual
        >>> import matplotlib.pyplot as plt
        >>> import numpy as np
        >>> fig = plt.figure()
        >>> pycapacity.visual.plot_polytope_faces([[6,5,4,3],[1,2,2,1]],plot=fig,face_color='blue')
        >>> pycapacity.visual.plot_polytope_faces([[10,8,7,6],[1,2,2,1]],plot=fig,face_color='red')
        >>> plt.show()

    Args:
        plot:  matplotlib ax to plot on, can be either ``matplotlib.pyplot`` or ``matplotlib.axes.Axes`` or ``matplotlib.figure.Figure`` or ``mpl_toolkits.mplot3d.axes3d.Axes3D``
        faces:  list of faces (optional **either vertex or polytope must be provided**)
        polytope(Polytope):  polytope object - if it is provided, it will use polytope.faces (optional **either vertex or polytope must be provided**)
        face_color:  polytope face color (optional)
        edge_color:  polytope edge color  (optional)
        alpha:  polytope opacity  (optional)
        label:  legend label (optional)
        center: offset the polytope (optional)
        scale: scale the polytope with a scalar (optional)
        wireframe: show the polytope as a wireframe (optional)

    Returns:
        ax:  matplotlib ax used for plotting
    """ 

    ax = None
    if faces is None and polytope is not None:
        faces = polytope.faces
    if faces is None:
        print("no data to plot")
        return ax


    # check if face shape is equal to 2 or 3 
    dim = np.array(faces).shape
    if 2 in dim:    dim = 2
    elif 3 in dim:  dim = 3



    # figure out what axes to plot on
    if plot == matplotlib.pyplot:
        # if its plt provided, get the current axes
        # see if the first one is already an axis, if not create one
        axes = plt.gcf().get_axes()
        if dim == 2:
            if len(axes) != 0 and isinstance(axes[0], matplotlib.axes.Axes):
                ax = axes[0]
            else:
                ax = plt.axes()
        else:
            if len(axes) != 0 and isinstance(axes[0], mpl_toolkits.mplot3d.axes3d.Axes3D):
                ax = axes[0]
                print("using existing 3d axis")
            else:
                ax = plt.axes(projection='3d')
                print("creating a new 3d axis")
    elif isinstance(plot, matplotlib.figure.Figure):
        # if its figure provided, get the current axes
        # see if the first one is already an axis, if not create one
        axes = plot.get_axes()
        if dim == 2:
            if len(axes) != 0 and isinstance(axes[0], matplotlib.axes.Axes):
                ax = axes[0]
            else:
                ax = plot.add_subplot(111)
        else:
            if len(axes) != 0  and isinstance(axes[0], mpl_toolkits.mplot3d.axes3d.Axes3D):
                print("using existing 3d axis")
                ax = axes[0]
            else:
                print("creating a new 3d axis")
                ax = plot.add_subplot(111, projection='3d')

    elif dim==2 and isinstance(plot, matplotlib.axes.Axes):
        #if its axes provided, use it
        ax = plot
    elif dim==3 and isinstance(plot, mpl_toolkits.mplot3d.axes3d.Axes3D):
        #if its axes provided, use it
        ax = plot
        print("using provided 3d axis")
    else:
        print(f"no matplotlib {dim}d axes provided")
        return ax

    # scale the data
    faces= np.array(faces)*scale

    if dim == 2:
        if center is None:
            center = (0,0)
        
        if wireframe:
            ax.fill(faces[0]+center[0],faces[1]+center[1], alpha=0.4, facecolor='none', edgecolor=edge_color, linewidth=3,label=label)
        else:
            ax.fill(faces[0]+center[0],faces[1]+center[1], alpha=0.4, facecolor=face_color, edgecolor=edge_color, linewidth=3,label=label)
    elif dim == 3:
        if center is None:
            center = (0,0,0)
        
        for polygone in faces:        
            try: # a small management of python3
                poly = Poly3DCollection([list(zip(polygone[0,:]+center[0],polygone[1,:]+center[1],polygone[2,:]+center[2]))])
            except ValueError: # vs python2
                poly = Poly3DCollection(list([zip(polygone[0,:]+center[0],polygone[1,:]+center[1],polygone[2,:]+center[2])]))
            
            if not wireframe:
                if alpha != None:
                    poly.set_alpha(alpha)
                if face_color != None:
                    poly.set_facecolors(face_color)
                if edge_color != None:
                    poly.set_edgecolor(edge_color)
            else:
                poly.set_facecolors((0,0,0,0))
                if edge_color != None:
                    poly.set_edgecolor(edge_color) # cannot set alpha for the moment
                else:
                    poly.set_edgecolor((0,0,0,alpha))
                
            ax.add_collection3d(poly)
        
        try: # a small management of python3
            poly._facecolors2d = poly._facecolor3d
            poly._edgecolors2d = poly._edgecolor3d
        except AttributeError: # vs python2
            poly._edgecolors2d=poly._edgecolors3d
            poly._facecolors2d=poly._facecolors3d
            
        if label != None:
            poly.set_label(label)
    else:
        print("cannot visualise data with dimension: "+str(dim))
    return ax

def plot_polytope_vertex(vertex=None, polytope=None, plot=None, label=None, color='black' , center=None, scale=1.0):
    """
    Polytope vertices plotting function in 2d and 3d

    Note:
        ``plot`` parameter can be either ``matplotlib.pyplot`` or ``matplotlib.axes.Axes`` or ``matplotlib.figure.Figure`` or ``mpl_toolkits.mplot3d.axes3d.Axes3D``.

    Examples:
        >>> import pycapacity.visual
        >>> import matplotlib.pyplot as plt
        >>> import numpy as np
        >>> fig = plt.figure()
        >>> pycapacity.visual.plot_polytope_vertex(np.array([[6,5,4,3],[1,2,2,1]]),plot=fig,color='blue')
        >>> pycapacity.visual.plot_polytope_vertex(np.array([[10,8,7,6],[1,2,2,1]]),plot=fig, color='red')
        >>> plt.show()
        
    Args:
        plot:  matplotlib ax to plot on, can be either ``matplotlib.pyplot`` or ``matplotlib.axes.Axes`` or ``matplotlib.figure.Figure`` or ``mpl_toolkits.mplot3d.axes3d.Axes3D``
        vertex :  vertices to be plotted (optional **either vertex or polytope must be provided**)
        polytope(Polytope):  polytope object - if it is provided, it will use the polytope vertices  (optional **either vertex or polytope must be provided**)
        color :  vertex color  (optional)
        label :  legend label (optional)
        center: offset the polytope (optional)
        scale: scale the polytope with a scalar (optional)

    Returns:
        ax :  matplotlib ax used for plotting
    """     


    ax = None
    if polytope is not None:
        vertex = polytope.vertices
    
    if polytope is None and vertex is None:
        print("no data to plot")
        return ax

    # check if face shape is equal to 2 or 3 
    dim = np.array(vertex).shape
    if 2 in dim:    dim = 2
    elif 3 in dim:  dim = 3


    # figure out what axes to plot on
    if plot == matplotlib.pyplot:
        # if its plt provided, get the current axes
        # see if the first one is already an axis, if not create one
        axes = plt.gcf().get_axes()
        if dim == 2:
            if len(axes) != 0 and isinstance(axes[0], matplotlib.axes.Axes):
                ax = axes[0]
            else:
                ax = plt.axes()
        else:
            if len(axes) != 0 and isinstance(axes[0], mpl_toolkits.mplot3d.axes3d.Axes3D):
                ax = axes[0]
            else:
                ax = plt.axes(projection='3d')
    elif isinstance(plot, matplotlib.figure.Figure):
        # if its figure provided, get the current axes
        # see if the first one is already an axis, if not create one
        axes = plot.get_axes()
        if dim == 2:
            if len(axes) != 0 and isinstance(axes[0], matplotlib.axes.Axes):
                ax = axes[0]
            else:
                ax = plot.add_subplot(111)
        else:
            if len(axes) != 0  and isinstance(axes[0], mpl_toolkits.mplot3d.axes3d.Axes3D):
                ax = axes[0]
            else:
                ax = plot.add_subplot(111, projection='3d')

    elif dim==2 and isinstance(plot, matplotlib.axes.Axes):
        #if its axes provided, use it
        ax = plot
    elif dim==3 and isinstance(plot, mpl_toolkits.mplot3d.axes3d.Axes3D):
        #if its axes provided, use it
        ax = plot
    else:
        print(f"no matplotlib {dim}d axes provided")
        return ax




    # scale the vertices
    vertex = np.array(vertex)*scale

    if dim == 2:
        if center is None:
            center = np.array([[0],[0]]).reshape(-1,1)
        else:
            center = np.array(center).reshape(-1,1)

        vertex = vertex + center
        if label != None:
            ax.scatter(vertex[0,:],vertex[1,:],color=color, label=label)
        else:
            ax.scatter(vertex[0,:],vertex[1,:],color=color)  
    elif dim == 3:
        if center is None:
            center = np.array([[0],[0],[0]])
        else:
            center = np.array(center).reshape(-1,1)
        

        vertex = vertex + center
        if label != None:
            ax.scatter(vertex[0,:],vertex[1,:],vertex[2,:],color=color, label=label)
        else:
            ax.scatter(vertex[0,:],vertex[1,:],vertex[2,:],color=color)  
    else:
        print("cannot visualise data with dimension: "+str(dim))
    return ax   

def plot_ellipsoid(radii=None, rotation=None, ellipsoid=None, center=None, plot=None, label=None, color=None, edge_color=None, alpha=1.0, scale=1.0):
    """
    Plotting ellipsoid in 2d and 3d

    Note:
        ``plot`` parameter can be either ``matplotlib.pyplot`` or ``matplotlib.axes.Axes`` or ``matplotlib.figure.Figure`` or ``mpl_toolkits.mplot3d.axes3d.Axes3D``.

    Examples:
        >>> import pycapacity.visual
        >>> import matplotlib.pyplot as plt
        >>> import numpy as np
        >>> fig = plt.figure()
        >>> pycapacity.visual.plot_ellipsoid([6,5,4],np.eye(3),plot=fig, color='blue', alpha=0.5)
        >>> pycapacity.visual.plot_ellipsoid([1,2,3],np.eye(3),plot=fig, color='red', alpha=0.5)
        >>> plt.show()
        
    Args:
        plot:  matplotlib ax to plot on, can be either ``matplotlib.pyplot`` or ``matplotlib.axes.Axes`` or ``matplotlib.figure.Figure`` or ``mpl_toolkits.mplot3d.axes3d.Axes3D``
        radii : radii of the ellipsoid in each axis (optional **either radii and rotation or ellipsoid must be provided**)
        rotation : rotation matrix (optional ***either radii and rotation or ellipsoid must be provided***)
        ellipsoid(Ellipsoid): ellipsoid object - if it is provided, it will use the ellipsoid radii and rotation (optional either radii and rotation or ellipsoid must be provided)
        center : offset of the ellipsoid from origin (optional)
        color :  face color (optional)
        edge_color : egde collor (optional)
        alpha : opacity (optional)
        label :  legend label (optional)
        scale: scale the polytope with a scalar (optional)

    Returns:
        ax :  matplotlib ax used for plotting
    """ 
    ax = None
    if ellipsoid is not None:
        radii = ellipsoid.radii
        rotation = ellipsoid.rotation
        if center is None:
            center = ellipsoid.center

    if ellipsoid is None and radii is None:
        print("no data to plot")
        return ax
    

    dim = np.array(radii).shape[0]
    U = rotation

    # scaling the ellipsoid
    radii = np.array(radii)*scale


    # figure out what axes to plot on
    if plot == matplotlib.pyplot:
        # if its plt provided, get the current axes
        # see if the first one is already an axis, if not create one
        axes = plt.gcf().get_axes()
        if dim == 2:
            if len(axes) != 0 and isinstance(axes[0], matplotlib.axes.Axes):
                ax = axes[0]
            else:
                ax = plt.axes()
        else:
            if len(axes) != 0 and isinstance(axes[0], mpl_toolkits.mplot3d.axes3d.Axes3D):
                ax = axes[0]
            else:
                ax = plt.axes(projection='3d')
    elif isinstance(plot, matplotlib.figure.Figure):
        # if its figure provided, get the current axes
        # see if the first one is already an axis, if not create one
        axes = plot.get_axes()
        if dim == 2:
            if len(axes) != 0 and isinstance(axes[0], matplotlib.axes.Axes):
                ax = axes[0]
            else:
                ax = plot.add_subplot(111)
        else:
            if len(axes) != 0  and isinstance(axes[0], mpl_toolkits.mplot3d.axes3d.Axes3D):
                ax = axes[0]
            else:
                ax = plot.add_subplot(111, projection='3d')

    elif dim==2 and isinstance(plot, matplotlib.axes.Axes):
        #if its axes provided, use it
        ax = plot
    elif dim==3 and isinstance(plot, mpl_toolkits.mplot3d.axes3d.Axes3D):
        #if its axes provided, use it
        ax = plot
    else:
        print(f"no matplotlib {dim}d axes provided")
        return ax



    if dim == 2:
        if center is None:
            center = (0,0)

        rx, ry = radii
        ellipse = Ellipse(xy=center, width=2*ry, height=2*rx, edgecolor=edge_color, fc='None', lw=2, angle=-np.arctan2(U[0,0],U[0,1])*180/np.pi)
        
        if label is not None:
            ellipse.set_label(label)

        ax.add_patch(ellipse)

    elif dim == 3:
        if center is None:
            center = (0,0,0)

        # Radii corresponding to the coefficients:
        rx, ry, rz = radii

        # Set of all spherical angles:
        u = np.linspace(0, 2 * np.pi, 30)
        v = np.linspace(0, np.pi, 30)

        # Cartesian coordinates that correspond to the spherical angles:
        # (this is the equation of an ellipsoid):
        x = rx * np.outer(np.cos(u), np.sin(v))
        y = ry * np.outer(np.sin(u), np.sin(v))
        z = rz * np.outer(np.ones_like(u), np.cos(v))

        for i in range(len(x)):
            for j in range(len(x)):
                [x[i,j],y[i,j],z[i,j]] = np.dot([x[i,j],y[i,j],z[i,j]], rotation.T) + center

        if label != None:
            surf = ax.plot_surface(x, y, z, color=color, alpha=alpha, edgecolor=edge_color, linewidth=1, label=label)
        else:
            surf = ax.plot_surface(x, y, z, color=color, alpha=alpha, edgecolor=edge_color, linewidth=1)

            
        try: # a small management of python3
            surf._facecolors2d = surf._facecolor3d
            surf._edgecolors2d = surf._edgecolor3d
        except AttributeError: # vs python2
            surf._edgecolors2d=surf._edgecolors3d
            surf._facecolors2d=surf._facecolors3d
  
    else:
        print("cannot visualise data with dimension: "+str(dim))
    return ax  


# see if mujoco is installed
try:
    import mujoco
    import mujoco.viewer
    MUJOCO_INSTALLED = True
except ImportError:
    MUJOCO_INSTALLED = False
    

if MUJOCO_INSTALLED:
    def mj_makeTriangle(v1,v2,v3 ,centroid):
        """
        Function scaling the unit triangle to match the triangle defined by the vertices v1, v2, v3
        and returning the scaling factors and the transformation matrix
        
        :param v1: vertex 1 of the triangle
        :param v2: vertex 2 of the triangle
        :param v3: vertex 3 of the triangle
        :param centroid: centroid of the triangle
        :return lengths: scaling factors for the edges and the normal    
        """
        e1 = v2-v1
        e2 = v3-v1
        normal = np.cross(e1,e2)    
        # check if normal is pointing towards the centroid
        if np.dot(normal, centroid-v1) > 0:
            # flip the normal and 
            normal = -normal
            tmp = e1
            e1 = e2
            e2 = tmp
            
        lengths = np.array([np.linalg.norm(e1),np.linalg.norm(e2),np.linalg.norm(normal)])
        xmat = np.array([e1/lengths[0], e2/lengths[1], normal/lengths[2]]).T
        return lengths, xmat
        

    # Function to draw force polytope
    def mj_draw_edges(viewer, faces, erase = True, color = [1, 0, 0, 0.5]):
        """ 
        Function that draws the edges of the polytope defined by its faces
        
        :param viewer: viewer object
        :param faces: faces of the polytope
        :param erase: if True, erase previous geometries
        :param color: color of the lines (RGBA, default is red [1, 0, 0, 0.5])
        """
        with viewer.lock():
            if erase:
                viewer.user_scn.ngeom = 0
            # Go through all faces of the convex hull
            for face in faces:
                p1, p2, p3 = face.T

                # Add line segments between hull points
                for start, end in [(p1, p2), (p2, p3), (p3, p1)]:
                    geom_id = viewer.user_scn.ngeom
                    if geom_id >= len(viewer.user_scn.geoms):
                        print("Too many faces, skipping the rest", geom_id)
                        continue  # Prevent out-of-bounds errors
                    
                    # avoid rendering the same line twice
                    if np.any(np.allclose(viewer.user_scn.geoms[geom_id].pos, start)):
                        continue
                    
                    mujoco.mjv_initGeom(
                        viewer.user_scn.geoms[geom_id],
                        int(mujoco.mjtGeom.mjGEOM_LINE),
                        np.array([0,0, np.linalg.norm(end - start)]),
                        start,
                        rotation_matrix_from_vectors([0, 0, 1], end - start).flatten(),
                        np.array(color)
                    )
                    viewer.user_scn.ngeom += 1  # Increase count
                    
                    
    # Function to draw force polytope
    def mj_draw_faces(viewer, simplices, centroid, erase = True, color = [1, 0, 0, 0.2]):
        """ 
        Function that draws the faces of the polytope defined by its simplices
        
        :param viewer: viewer object
        :param simplices: simplices of the polytope
        :param centroid: centroid of the polytope
        :param erase: if True, erase previous geometries 
        :param color: color of the faces (RGBA, default is red [1, 0, 0, 0.5])
        """

        with viewer.lock():
            if erase:
                viewer.user_scn.ngeom = 0
            
            # Go through all faces of the convex hull
            for simplex in simplices:
                p1, p2, p3 = simplex.T

                geom_id = viewer.user_scn.ngeom
                if geom_id >= len(viewer.user_scn.geoms):
                    print("Too many faces, skipping the rest", geom_id)
                    break  # Prevent out-of-bounds errors
                
                        
                size, mat = mj_makeTriangle(p1, p2, p3, centroid)
                
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[geom_id],
                    int(mujoco.mjtGeom.mjGEOM_TRIANGLE),
                    size,
                    p1,
                    mat.flatten(),
                    np.array(color)
                )
                # change to the material of the object
                viewer.user_scn.geoms[geom_id].emission = 0.9
                viewer.user_scn.ngeom += 1  # Increase count
                

    def mj_draw_polytope(viewer, polytope, edges=True, faces=True, color = [1, 0, 0, 0.2]):
        """
        Funciton that draws the polytope defined by its vertices and faces 
        
        :param viewer: viewer object
        :param polytope: polytope object
        :param eedges: if True, draw the edges of the polytope
        :param faces: if True, draw the faces of the polytope
        :param color: color of the polytope (RGBA, default is red [1, 0, 0, 0.5])
        """
        
        if polytope.vertices.shape[1] < 4:
            return
        
        # if faces not computed yet, compute them
        if polytope.faces is None:
            polytope.find_faces()
        
        # Draw the polytope
        if edges:
            mj_draw_edges(viewer, polytope.faces, erase = True, color = color)
        if faces:
            x_center = np.mean(polytope.vertices, axis=1)
            mj_draw_faces(viewer, polytope.faces, x_center, erase = not edges, color = color)