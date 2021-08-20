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

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L84"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L123"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L211"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L250"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L291"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `velocity_polytope`

```python
velocity_polytope(Jacobian, dq_max, dq_min, gravity=None)
```

Velocity polytope calculating function 



**Args:**
 
 - <b>`Jacobian`</b>:   position jacobian  
 - <b>`dq_max`</b>:   maximal joint velocities  
 - <b>`dq_min`</b>:   minimal joint velocities  



**Returns:**
 
 - <b>`velocity_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L306"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `velocity_polytope_withfaces`

```python
velocity_polytope_withfaces(Jacobian, dq_max, dq_min, gravity=None)
```

Velocity polytope calculating function, with faces 



**Args:**
 
 - <b>`Jacobian`</b>:   position jacobian  
 - <b>`dq_max`</b>:   maximal joint velocities  
 - <b>`dq_min`</b>:   minimal joint velocities  



**Returns:**
 
 - <b>`velocity_vertex`</b> (list):   vertices of the polytope 
 - <b>`faces`</b> (list):   faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L325"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `hyper_plane_shift_method`

```python
hyper_plane_shift_method(A, x_min, x_max, tol=1e-15)
```

Hyper plane shifting method implementation used to solve problems of a form: y = Ax s.t. x_min <= x <= x_max 

Hyperplane shifting method:  *Gouttefarde M., Krut S. (2010) Characterization of Parallel Manipulator Available Wrench Set Facets. In: Lenarcic J., Stanisic M. (eds) Advances in Robot Kinematics: Motion in Man and Machine. Springer, Dordrecht* 



This algorithm can be used to calcualte acceleration polytope, velocity polytoe and even  polytope of the joint achievable joint torques based on the muscle forces 



**Args:**
 
 - <b>`A`</b>:  projection matrix 
 - <b>`x_min`</b>:  minimal values 
 - <b>`x_max`</b>:  maximal values  



**Returns:**
 
 - <b>`H`</b>:  half space representation matrix H - Hx < d 
 - <b>`d`</b>:  half space representaiton vector d - Hx < d 
 - <b>`vertices`</b>:  vertex representation of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L399"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L424"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L440"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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
