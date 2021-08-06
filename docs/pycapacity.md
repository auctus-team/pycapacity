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
 
 - <b>`param1`</b>:  Jacobian_position position jacobian 
 - <b>`param2`</b>:  dq_max maximal joint velocities 

**Returns:**
 
 - <b>`list`</b>:   list of singular values S 
 - <b>`array`</b>:  the matrix U 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L33"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `manipulability_force`

```python
manipulability_force(Jacobian_position, t_max)
```

force manipulability calculation 

**Args:**
 
 - <b>`param1`</b>:  Jacobian_position position jacobian 
 - <b>`param2`</b>:  t_max maximal joint torques 

**Returns:**
 
 - <b>`list`</b>:   list of singular values 1/S 
 - <b>`array`</b>:  the matrix U 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L53"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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
 
 - <b>`param1`</b>:  Jacobian1 position jacobian robot 1 
 - <b>`param2`</b>:  Jacobian2 position jacobian robot 2 
 - <b>`param3`</b>:  t_max1 maximal joint torques robot 1 
 - <b>`param4`</b>:  t_max2 maximal joint torques robot 2 
 - <b>`param5`</b>:  gravity1 applied joint torques (for example gravity vector  or J^T*f ) robot 1 
 - <b>`param6`</b>:  gravity2 maximal joint torques (for example gravity vector  or J^T*f ) robot 2 



**Returns:**
 
 - <b>`list`</b>:  f_vertex vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L81"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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
 
 - <b>`param1`</b>:  Jacobian1 position jacobian robot 1 
 - <b>`param2`</b>:  Jacobian2 position jacobian robot 2 
 - <b>`param3`</b>:  t_max1 maximal joint torques robot 1 
 - <b>`param4`</b>:  t_max2 maximal joint torques robot 2 
 - <b>`param5`</b>:  gravity1 applied joint torques (for example gravity vector  or J^T*f ) robot 1 
 - <b>`param6`</b>:  gravity2 maximal joint torques (for example gravity vector  or J^T*f ) robot 2 



**Returns:**
 
 - <b>`list`</b>:  f_vertex vertices of the polytope 
 - <b>`list`</b>:  polytope_faces faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L118"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope`

```python
force_polytope(Jacobian, t_max, t_min, gravity=None)
```

Force polytope representing the capacities of the two robots in a certain configuration 



**Args:**
 
 - <b>`param1`</b>:  Jacobian1 position jacobian  
 - <b>`param3`</b>:  t_max1 maximal joint torques  
 - <b>`param5`</b>:  gravity1 applied joint torques (for example gravity vector  or J^T*f )   



**Returns:**
 
 - <b>`list`</b>:  f_vertex vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L205"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_withfaces`

```python
force_polytope_withfaces(Jacobian, t_max, t_min, gravity=None)
```

Force polytope representing the capacities of the two robots in a certain configuration. With vertices ordered into the faces 



**Args:**
 
 - <b>`param1`</b>:  Jacobian1 position jacobian  
 - <b>`param3`</b>:  t_max1 maximal joint torques  
 - <b>`param5`</b>:  gravity1 applied joint torques (for example gravity vector  or J^T*f )   



**Returns:**
 
 - <b>`list`</b>:  f_vertex vertices of the polytope 
 - <b>`list`</b>:  polytope_faces faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L243"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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
 
 - <b>`param1`</b>:  Jacobian1 position jacobian robot 1 
 - <b>`param2`</b>:  Jacobian2 position jacobian robot 2 
 - <b>`param3`</b>:  t_max1 maximal joint torques robot 1 
 - <b>`param4`</b>:  t_max2 maximal joint torques robot 2 
 - <b>`param5`</b>:  gravity1 applied joint torques (for example gravity vector  or J^T*f ) robot 1 
 - <b>`param6`</b>:  gravity2 maximal joint torques (for example gravity vector  or J^T*f ) robot 2 



**Returns:**
 
 - <b>`list`</b>:  f_vertex vertices of the polytope 
 - <b>`list`</b>:  polytope_faces faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L282"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `make_2d`

```python
make_2d(points)
```

Take a list of 3D(cooplanar) points and make it 2D 

**Args:**
 
 - <b>`param1`</b>:  points matrix of 3D points 

**Returns:**
 
 - <b>`array`</b>:   list array of 2D points 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L307"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `order_index`

```python
order_index(points)
```

Order clockwise 2D points 

**Args:**
 
 - <b>`param1`</b>:  points matrix of 2D points 

**Returns:**
 
 - <b>`array`</b>:  ordered indexes 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/pycapacity/pycapacity.py#L323"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `make_unique`

```python
make_unique(points)
```

Remove repetitions of columns 



**Args:**
 
 - <b>`param1`</b>:  points matrix of n-dim points 

**Returns:**
 
 - <b>`array`</b>:  matrix with only unique pints 




---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
