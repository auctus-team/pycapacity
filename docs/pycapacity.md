<!-- markdownlint-disable -->

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L0"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

# <kbd>module</kbd> `pycapacity`





---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L12"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `manipulability_velocity`

```python
manipulability_velocity(Jacobian_position, dq_max)
```

velocity manipulability calculation 



**Args:**
 
 - <b>`Jacobian_position`</b>:  position jacobian 
 - <b>`dq_max`</b>:   maximal joint velocities 

**Returns:**
 
 - <b>`S`</b> (list):   list of singular values S 
 - <b>`U`</b> (matrix):  the matrix U 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L33"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `manipulability_force`

```python
manipulability_force(Jacobian_position, t_max)
```

force manipulability calculation 



**Args:**
 
 - <b>`Jacobian_position`</b>:  position jacobian 
 - <b>`dq_max`</b>:   maximal joint velocities 

**Returns:**
 
 - <b>`list`</b>:   list of singular values 1/S 
 - <b>`U`</b> (matrix):  the matrix U 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L54"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_intersection`

```python
force_polytope_intersection(
    Jacobian1,
    Jacobian2,
    t1_max,
    t1_min,
    t2_max,
    t2_min,
    gravity1,
    gravity2
)
```

Force polytope representing the intersection of the capacities of the two robots in certain configurations. 





**Args:**
 
 - <b>`Jacobian1`</b>:   position jacobian robot 1 
 - <b>`Jacobian2`</b>:  Jacobian2 position jacobian robot 2 
 - <b>`t_min1`</b>:   minimal joint torques robot 1 
 - <b>`t_min2`</b>:   minimal joint torques robot 2 
 - <b>`t_max1`</b>:   maximal joint torques robot 1 
 - <b>`t_max2`</b>:   maximal joint torques robot 2 
 - <b>`gravity1`</b>:   applied joint torques (for example gravity vector  or J^T*f ) robot 1 
 - <b>`gravity2`</b>:   maximal joint torques (for example gravity vector  or J^T*f ) robot 2 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L79"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_sum_withfaces`

```python
force_polytope_sum_withfaces(
    Jacobian1,
    Jacobian2,
    t1_max,
    t1_min,
    t2_max,
    t2_min,
    gravity1=None,
    gravity2=None
)
```

Force polytope representing the minkowski sum of the capacities of the two robots in certain configurations. With ordered vertices into the faces. 



**Args:**
 
 - <b>`Jacobian1`</b>:   position jacobian robot 1 
 - <b>`Jacobian2`</b>:  Jacobian2 position jacobian robot 2 
 - <b>`t_min1`</b>:   minimal joint torques robot 1 
 - <b>`t_min2`</b>:   minimal joint torques robot 2 
 - <b>`t_max1`</b>:   maximal joint torques robot 1 
 - <b>`t_max2`</b>:   maximal joint torques robot 2 
 - <b>`gravity1`</b>:   applied joint torques (for example gravity vector  or J^T*f ) robot 1 
 - <b>`gravity2`</b>:   maximal joint torques (for example gravity vector  or J^T*f ) robot 2 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 
 - <b>`faces`</b> (list):  polytope_faces faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L118"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope`

```python
force_polytope(Jacobian, t_max, t_min, gravity=None)
```

Force polytope representing the capacities of the two robots in a certain configuration 



**Args:**
 
 - <b>`Jacobian`</b>:   position jacobian  
 - <b>`t_max`</b>:   maximal joint torques  
 - <b>`t_min`</b>:   minimal joint torques  
 - <b>`gravity`</b>:   applied joint torques (for example gravity vector  or J^T*f )   



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L206"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_withfaces`

```python
force_polytope_withfaces(Jacobian, t_max, t_min, gravity=None)
```

Force polytope representing the capacities of the two robots in a certain configuration. With vertices ordered into the faces 



**Args:**
 
 - <b>`Jacobian`</b>:   position jacobian  
 - <b>`t_max`</b>:   maximal joint torques  
 - <b>`t_min`</b>:   minimal joint torques  
 - <b>`gravity`</b>:   applied joint torques (for example gravity vector  or J^T*f )   



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 
 - <b>`faces`</b> (list):   faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L245"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_intersection_withfaces`

```python
force_polytope_intersection_withfaces(
    Jacobian1,
    Jacobian2,
    t1_max,
    t1_min,
    t2_max,
    t2_min,
    gravity1=None,
    gravity2=None
)
```

Force polytope representing the intersection of the capacities of the two robots in certain configurations. With ordered vertices into the faces. 



**Args:**
 
 - <b>`Jacobian1`</b>:   position jacobian robot 1 
 - <b>`Jacobian2`</b>:  Jacobian2 position jacobian robot 2 
 - <b>`t_min1`</b>:   minimal joint torques robot 1 
 - <b>`t_min2`</b>:   minimal joint torques robot 2 
 - <b>`t_max1`</b>:   maximal joint torques robot 1 
 - <b>`t_max2`</b>:   maximal joint torques robot 2 
 - <b>`gravity1`</b>:   applied joint torques (for example gravity vector  or J^T*f ) robot 1 
 - <b>`gravity2`</b>:   maximal joint torques (for example gravity vector  or J^T*f ) robot 2 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 
 - <b>`faces`</b> (list):  polytope_faces faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L286"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `make_2d`

```python
make_2d(points)
```

Take a list of 3D(cooplanar) points and make it 2D 

**Args:**
 
 - <b>`points3D`</b>:   matrix of 3D points 

**Returns:**
 
 - <b>`points2D`</b> (array):   list array of 2D points 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L311"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `order_index`

```python
order_index(points)
```

Order clockwise 2D points 

**Args:**
 
 - <b>`points`</b>:   matrix of 2D points 

**Returns:**
 
 - <b>`indexes`</b> (array):  ordered indexes 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L327"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `make_unique`

```python
make_unique(points)
```

Remove repetitions of columns 



**Args:**
 
 - <b>`points`</b>:   matrix of n-dim points 

**Returns:**
 
 - <b>`unique`</b>:  matrix with only unique pints 




---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
