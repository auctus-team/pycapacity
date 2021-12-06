<!-- markdownlint-disable -->

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/human.py#L0"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

# <kbd>module</kbd> `human`





---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/human.py#L10"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `joint_torques_polytope`

```python
joint_torques_polytope(N, F_min, F_max, tol=1e-15)
```

A function calculating the polytopes of achievable joint torques based on the moment arm matrix N : 

t = N.F st F_min <= F <= F_max 



**Args:**
 
 - <b>`N`</b>:  moment arm matrix 
 - <b>`F_min`</b>:  minimal muscular forces (passive forces or 0) 
 - <b>`F_max`</b>:  maximal isometric forces  
 - <b>`tolerance`</b>:  tolerance for the polytope calculation 



**Returns:**
 
 - <b>`t_vert`</b> (array):   list of torque vertices 
 - <b>`H`</b> (array):   half-space rep matrix H - H.t < d 
 - <b>`d`</b> (array):   half-space rep vector d 
 - <b>`faces`</b>:  indexes of verteices forming the polytope faces 


---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/human.py#L32"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `acceleration_polytope`

```python
acceleration_polytope(J, N, M, F_min, F_max, tol=1e-15)
```

A function calculating the polytopes of achievable accelerations based on the jacobian matrix J, moment arm matrix N and mass matrix M 

a = ddx = J.M^(-1).N.F st F_min <= F <= F_max 



**Args:**
 
 - <b>`J`</b>:  jacobian matrix 
 - <b>`N`</b>:  moment arm matrix 
 - <b>`M`</b>:  mass matrix 
 - <b>`F_min`</b>:  minimal muscular forces (passive forces or 0) 
 - <b>`F_max`</b>:  maximal isometric forces  
 - <b>`tolerance`</b>:  tolerance for the polytope calculation 



**Returns:**
 
 - <b>`a_vert`</b> (array):   list of acceleraiton vertices 
 - <b>`H`</b> (array):   half-space rep matrix H - H.a < d 
 - <b>`d`</b> (array):   half-space rep vectors d 
 - <b>`faces`</b>:  indexes of verteices forming the polytope faces 


---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/human.py#L56"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `force_polytope`

```python
force_polytope(J, N, F_min, F_max, tol, torque_bias=None)
```

A function calculating the polytopes of achievable foreces based  on the jacobian matrix J and moment arm matrix N 

J^T.f = N.F (+ t_bias  optional)  st F_min <= F <= F_max 



**Args:**
 
 - <b>`J`</b>:  jacobian matrix 
 - <b>`N`</b>:  moment arm matrix 
 - <b>`F_min`</b>:  minimal muscular forces (passive forces or 0) 
 - <b>`F_max`</b>:  maximal isometric forces  
 - <b>`tolerance`</b>:  tolerance for the polytope calculation 
 - <b>`torque_bias`</b>:  torque bias optional (gravity or movement or applied forces ....)  



**Returns:**
 
 - <b>`f_vert`</b> (list):   list of cartesian force vertices 
 - <b>`H`</b> (array):   half-space rep matrix H - H.a < d 
 - <b>`d`</b> (array):   half-space rep vectors d 
 - <b>`faces`</b> (list):    list of vertex indexes forming polytope faces   


---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/human.py#L80"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `velocity_polytope`

```python
velocity_polytope(J, N, dl_min, dl_max, tol)
```

A function calculating the polytopes of achievable velocity based  on the jacobian matrix J and moment arm matrix N 

L.q = dl J.q = v st dl_min <= dl <= dl_max 



**Args:**
 
 - <b>`J`</b>:  jacobian matrix 
 - <b>`N`</b>:  moment arm matrix L = -N^T 
 - <b>`dl_min`</b>:  minimal achievable muscle contraction veclocity 
 - <b>`dl_max`</b>:  maximal achievable muscle contraction veclocity 
 - <b>`tolerance`</b>:  tolerance for the polytope calculation 



**Returns:**
 
 - <b>`v_vert`</b> (list):   list of cartesian velocity vertices 
 - <b>`H`</b> (array):   half-space rep matrix H - H.a < d 
 - <b>`d`</b> (array):   half-space rep vectors d 
 - <b>`faces`</b> (list):    list of vertex indexes forming velocity polytope faces   


---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/human.py#L104"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `torque_to_muscle_force`

```python
torque_to_muscle_force(N, F_min, F_max, tau, options='lp')
```

A function calculating muscle forces needed to create the joint torques tau 



**Args:**
 
 - <b>`N`</b>:  moment arm matrix 
 - <b>`F_min`</b>:  minimal muscular forces (passive forces or 0) 
 - <b>`F_max`</b>:  maximal isometric forces  
 - <b>`tau`</b>:  joint torque 
 - <b>`options`</b>:  solver type to use: 'lp' - linear programming (defualt), 'qp' - quadratic programming 



**Returns:**
 
 - <b>`F`</b> (list):  list of muscle forces 






---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
