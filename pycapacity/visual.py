import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Ellipse
import numpy as np

def plot_polytope_faces(faces,ax=None, plt=None, face_color=None, edge_color=None, alpha=None,label=None, center=None):
    """ 
    Polytope faces plotting function in 2d and 3d. 
    
    Note:
        If provided matplotlib `plt` it will find the `ax` automatically. No need to provide the `ax`. If `ax` already defined no need to provide the `plt`.

    Examples:
        >>> import pycapacity.visual
        >>> import matplotlib.pyplot as plt
        >>> import numpy as np
        >>> ax = pycapacity.visual.plot_polytope_faces([[6,5,4,3],[1,2,2,1]],plt=plt,face_color='blue')
        >>> pycapacity.visual.plot_polytope_faces([[10,8,7,6],[1,2,2,1]],ax=ax,face_color='red')
        >>> plt.show()

    Args:
        faces:  list of faces (vertices)
        ax:  matplotlib ax to plot on 
        plt: matplotlib plot to plot on - it will find the ax automatically
        face_color:  polytope face color 
        edge_color:  polytope edge color 
        alpha:  polytope opacity 
        label:  legend label
        center: offset the polytope

    Returns:
        ax:  matplotlib ax used for plotting
    """ 
    dim = min(np.array(faces).shape)
    if dim == 2:
        if center is None:
            center = (0,0)
        if ax is None:
            ax = plt.axes()
        ax.fill(faces[0]+center[0],faces[1]+center[1], alpha=0.4, facecolor=face_color, edgecolor=edge_color, linewidth=3,label=label)
    elif dim == 3:
        if center is None:
            center = (0,0,0)
        if ax is None:
            ax = plt.axes(projection='3d')
        for polygone in faces:        
            try: # a small management of python3
                poly = Poly3DCollection([list(zip(polygone[0,:]+center[0],polygone[1,:]+center[1],polygone[2,:]+center[2]))])
            except ValueError: # vs python2
                poly = Poly3DCollection(list([zip(polygone[0,:]+center[0],polygone[1,:]+center[1],polygone[2,:]+center[2])]))
                
            if alpha != None:
                poly.set_alpha(alpha)
            if face_color != None:
                poly.set_facecolor(face_color)
            if edge_color != None:
                poly.set_edgecolor(edge_color)
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

def plot_polytope_vertex(vertex, ax=None, plt=None, label=None, color='black' , center=None):
    """
    Polytope vertices plotting function in 2d and 3d

    Note:
        If provided matplotlib `plt` it will find the `ax` automatically. No need to provide the `ax`. If `ax` already defined no need to provide the `plt`.

    Examples:
        >>> import pycapacity.visual
        >>> import matplotlib.pyplot as plt
        >>> import numpy as np
        >>> ax = pycapacity.visual.plot_polytope_vertex(np.array([[6,5,4,3],[1,2,2,1]]),plt=plt,color='blue')
        >>> pycapacity.visual.plot_polytope_vertex(np.array([[10,8,7,6],[1,2,2,1]]),ax=ax, color='red')
        >>> plt.show()
        
    Args:
        vertex :  position jacobian 
        ax :  matplotlib ax to plot on 
        plt : matplotlib plot to plot on - it will find the ax automatically
        color :  vertex color 
        label :  legend label
        center: offset the polytope

    Returns:
        ax :  matplotlib ax used for plotting
    """ 
    dim = np.array(vertex).shape[0]
    if dim == 2:
        if center is None:
            center = np.array([[0],[0]]).reshape(-1,1)
        else:
            center = np.array(center).reshape(-1,1)

        if ax is None:
            ax = plt.axes()

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
        if ax is None:
            ax = plt.axes(projection='3d')
        vertex = vertex + center
        if label != None:
            ax.scatter(vertex[0,:],vertex[1,:],vertex[2,:],color=color, label=label)
        else:
            ax.scatter(vertex[0,:],vertex[1,:],vertex[2,:],color=color)  
    else:
        print("cannot visualise data with dimension: "+str(dim))
    return ax   


def plot_ellipsoid(radii, rotation, center=None, ax=None, plt=None, label=None, color=None, edge_color=None, alpha=1.0):
    """
    Plotting ellipsoid in 2d and 3d

    Note:
        If provided matplotlib `plt` it will find the `ax` automatically. No need to provide the `ax`. If `ax` already defined no need to provide the `plt`.

    Examples:
        >>> import pycapacity.visual
        >>> import matplotlib.pyplot as plt
        >>> import numpy as np
        >>> ax = pycapacity.visual.plot_ellipsoid([6,5,4],np.eye(3),plt=plt, color='blue', alpha=0.5)
        >>> pycapacity.visual.plot_ellipsoid([1,2,3],np.eye(3),ax=ax, color='red', alpha=0.5)
        >>> plt.show()
        
    Args:
        radii : radii of the ellipsoid in each axis
        rotation : rotation matrix 
        center : offset of the ellispoid from origin
        ax :  matplotlib ax to plot on 
        plt : matplotlib plot to plot on - it will find the ax automatically
        color :  face color 
        edge_color : egde collor
        alpha : opacity
        label :  legend label

    Returns:
        ax :  matplotlib ax used for plotting
    """ 
    dim = np.array(radii).shape[0]
    U = rotation

    if dim == 2:
        if center is None:
            center = (0,0)

        if ax is None:
            ax = plt.axes()

        rx, ry = radii
        ellipse = Ellipse(xy=center, width=2*ry, height=2*rx, edgecolor=edge_color, fc='None', lw=2, angle=-np.arctan2(U[0,0],U[0,1])*180/np.pi)
        
        if label is not None:
            ellipse.set_label(label)

        ax.add_patch(ellipse)

    elif dim == 3:
        if center is None:
            center = (0,0,0)

        if ax is None:
            ax = plt.axes(projection='3d')

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