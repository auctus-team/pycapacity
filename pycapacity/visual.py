import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

def plot_polytope_faces(faces,ax=None, plt=None, face_color=None, edge_color=None, alpha=None,label=None):
    """
    Polytope faces plotting function in 2d and 3d

    Args:
        faces:  list of faces (vertices)
        ax:  matplotlib ax to plot on 
        plt: matplotlib plot to plot on - it will find the ax automatically
        face_color:  polytope face color 
        edge_color:  polytope edge color 
        alpha:  polytope opacity 
        label:  legend label

    Returns:
        ax:  matplotlib ax used for plotting
    """ 
    dim = min(np.array(faces).shape)
    if dim == 2:
        if ax is None:
            ax = plt.axes()
        ax.fill(faces[0],faces[1], alpha=0.4, facecolor=face_color, edgecolor=edge_color, linewidth=3,label=label)
    elif dim == 3:
        if ax is None:
            ax = plt.axes(projection='3d')
        for polygone in faces:        
            try: # a small management of python3
                poly = Poly3DCollection([list(zip(polygone[0,:],polygone[1,:],polygone[2,:]))])
            except ValueError: # vs python2
                poly = Poly3DCollection(list([zip(polygone[0,:],polygone[1,:],polygone[2,:])]))
                
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

def plot_polytope_vertex(vertex,ax=None, plt=None, label=None, color='black'):
    """
    Polytope vertices plotting function in 2d and 3d

    Args:
        vertex:  position jacobian 
        ax:  matplotlib ax to plot on 
        plt: matplotlib plot to plot on - it will find the ax automatically
        color:  vertex color 
        label:  legend label

    Returns:
        ax:  matplotlib ax used for plotting
    """ 
    dim = np.array(vertex).shape[0]
    if dim == 2:
        if ax is None:
            ax = plt.axes()
        if label != None:
            ax.scatter(vertex[0,:],vertex[1,:],color=color, label=label)
        else:
            ax.scatter(vertex[0,:],vertex[1,:],color=color)  
    elif dim == 3:
        if ax is None:
            ax = plt.axes(projection='3d')
        if label != None:
            ax.scatter(vertex[0,:],vertex[1,:],vertex[2,:],color=color, label=label)
        else:
            ax.scatter(vertex[0,:],vertex[1,:],vertex[2,:],color=color)  
    else:
        print("cannot visualise data with dimension: "+str(dim))
    return ax   