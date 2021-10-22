<!-- markdownlint-disable -->

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L0"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

# <kbd>module</kbd> `pycapacity`





---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L9"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `velocity_ellipsoid`

```python
velocity_ellipsoid(J, dq_max)
```

velocity manipulability ellipsoid calculation 



**Args:**
 
 - <b>`J`</b>:  position jacobian 
 - <b>`dq_max`</b>:   maximal joint velocities 

**Returns:**
 
 - <b>`S`</b> (list):   list of axis lengths 
 - <b>`U`</b> (matrix):  list of axis vectors 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L30"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `acceleration_ellipsoid`

```python
acceleration_ellipsoid(J, M, t_max)
```

acceleration ellipsoid calculation (dynamic manipulability ellipsoid) 



**Args:**
 
 - <b>`J`</b>:  matrix jacobian 
 - <b>`M`</b>:  matrix inertia  
 - <b>`t_max`</b>:   maximal joint torques 

**Returns:**
 
 - <b>`S`</b> (list):   list of axis lengths 
 - <b>`U`</b> (matrix):  list of axis vectors 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L52"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_ellipsoid`

```python
force_ellipsoid(J, t_max)
```

force manipulability ellipsoid calculation 



**Args:**
 
 - <b>`J`</b>:  matrix jacobian 
 - <b>`t_max`</b>:   maximal joint torques 

**Returns:**
 
 - <b>`S`</b> (list):   list of axis lengths 
 - <b>`U`</b> (matrix):  list of axis vectors 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L73"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_intersection`

```python
force_polytope_intersection(
    Jacobian1,
    Jacobian2,
    t1_max,
    t1_min,
    t2_max,
    t2_min,
    t1_bias,
    t2_bias
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
 - <b>`t1_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1 
 - <b>`t2_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L102"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_sum_withfaces`

```python
force_polytope_sum_withfaces(
    Jacobian1,
    Jacobian2,
    t1_max,
    t1_min,
    t2_max,
    t2_min,
    t1_bias=None,
    t2_bias=None
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
 - <b>`t1_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1 
 - <b>`t2_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 
 - <b>`faces`</b> (list):  polytope_faces faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L141"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope`

```python
force_polytope(Jacobian, t_max, t_min, t_bias=None)
```

Force polytope representing the capacities of the two robots in a certain configuration 



**Args:**
 
 - <b>`Jacobian`</b>:   position jacobian  
 - <b>`t_max`</b>:   maximal joint torques  
 - <b>`t_min`</b>:   minimal joint torques  
 - <b>`t_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot  



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L229"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_withfaces`

```python
force_polytope_withfaces(Jacobian, t_max, t_min, t_bias=None)
```

Force polytope representing the capacities of the two robots in a certain configuration. With vertices ordered into the faces 



**Args:**
 
 - <b>`Jacobian`</b>:   position jacobian  
 - <b>`t_max`</b>:   maximal joint torques  
 - <b>`t_min`</b>:   minimal joint torques  
 - <b>`t_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 
 - <b>`faces`</b> (list):   faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L269"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope_intersection_withfaces`

```python
force_polytope_intersection_withfaces(
    Jacobian1,
    Jacobian2,
    t1_max,
    t1_min,
    t2_max,
    t2_min,
    t1_bias=None,
    t2_bias=None
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
 - <b>`t1_bias`</b>:   bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 1 
 - <b>`t2_bias`</b>:   bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces for robot 2 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 
 - <b>`faces`</b> (list):  polytope_faces faces of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L310"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `velocity_polytope`

```python
velocity_polytope(Jacobian, dq_max, dq_min)
```

Velocity polytope calculating function 



**Args:**
 
 - <b>`Jacobian`</b>:   position jacobian  
 - <b>`dq_max`</b>:   maximal joint velocities  
 - <b>`dq_min`</b>:   minimal joint velocities  



**Returns:**
 
 - <b>`velocity_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L325"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `velocity_polytope_withfaces`

```python
velocity_polytope_withfaces(Jacobian, dq_max, dq_min)
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

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L344"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `acceleration_polytope`

```python
acceleration_polytope(J, M, t_max, t_min, t_bias=None)
```

Acceleration polytope calculating function 



**Args:**
 
 - <b>`J`</b>:   position jacobian  
 - <b>`M`</b>:   inertia matrix  
 - <b>`t_max`</b>:   maximal joint torque  
 - <b>`t_min`</b>:   minimal joint torque  
 - <b>`t_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces 

**Returns:**
 
 - <b>`acceleration_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L364"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `acceleration_polytope_withfaces`

```python
acceleration_polytope_withfaces(Jacobian, dq_max, dq_min, t_bias=None)
```

Acceleration polytope calculating function 



**Args:**
 
 - <b>`J`</b>:   position jacobian  
 - <b>`M`</b>:   inertia matrix  
 - <b>`t_max`</b>:   maximal joint torque  
 - <b>`t_min`</b>:   minimal joint torque  
 - <b>`t_bias`</b>:  bias joint torques due to the gravity, robot dynamics and maybe some already appiled forces 

**Returns:**
 
 - <b>`acceleration_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L389"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L462"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L487"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/askuric/pycapacity/-/blob/master/pycapacity.py#L503"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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
