<!-- markdownlint-disable -->

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/visual.py#L0"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

# <kbd>module</kbd> `visual`





---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/visual.py#L5"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `plot_polytope_faces`

```python
plot_polytope_faces(
    faces,
    ax=None,
    plt=None,
    face_color=None,
    edge_color=None,
    alpha=None,
    label=None
)
```

Polytope faces plotting function in 2d and 3d 



**Args:**
 
 - <b>`faces`</b>:   list of faces (vertices) 
 - <b>`ax`</b>:   matplotlib ax to plot on  
 - <b>`plt`</b>:  matplotlib plot to plot on - it will find the ax automatically 
 - <b>`face_color`</b>:   polytope face color  
 - <b>`edge_color`</b>:   polytope edge color  
 - <b>`alpha`</b>:   polytope opacity  
 - <b>`label`</b>:   legend label 



**Returns:**
 
 - <b>`ax`</b>:   matplotlib ax used for plotting 


---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/visual.py#L56"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `plot_polytope_vertex`

```python
plot_polytope_vertex(vertex, ax=None, plt=None, label=None, color='black')
```

Polytope vertices plotting function in 2d and 3d 



**Args:**
 
 - <b>`vertex`</b>:   position jacobian  
 - <b>`ax`</b>:   matplotlib ax to plot on  
 - <b>`plt`</b>:  matplotlib plot to plot on - it will find the ax automatically 
 - <b>`color`</b>:   vertex color  
 - <b>`label`</b>:   legend label 



**Returns:**
 
 - <b>`ax`</b>:   matplotlib ax used for plotting 




---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
