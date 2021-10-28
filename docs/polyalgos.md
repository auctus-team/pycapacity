<!-- markdownlint-disable -->

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L0"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

# <kbd>module</kbd> `polyalgos`





---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L11"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `iterative_convex_hull_method`

```python
iterative_convex_hull_method(A, B, y_min, y_max, tol, P=None)
```

A function calculating the polytopes of achievable x for equations form: 

z = B.y A.x = z s.t. y_min <= y <= y_max 

or 

A.x = B.y s.t. y_min <= y <= y_max 

(optionally - additional projection matrix) A.z = B.y P.z = x s.t. y_min <= y <= y_max 

On-line feasible wrench polytope evaluation based on human musculoskeletal models: an iterative convex hull method A.Skuric,V.Padois,N.Rezzoug,D.Daney  



**Args:**
 
 - <b>`A`</b>:  matrix 
 - <b>`B`</b>:  matrix 
 - <b>`y_min`</b>:  minimal values 
 - <b>`y_max`</b>:  maximal values 
 - <b>`tol`</b>:  tolerance for the polytope calculation 
 - <b>`P`</b>:  an additional projection matrix  



**Returns:**
 
 - <b>`x_vert`</b> (list):   list of cartesian force vertices 
 - <b>`H`</b> (list):   matrix of half-space representation Hx<d 
 - <b>`d`</b> (list):   vector of half-space representation Hx<d 
 - <b>`faces`</b> (list):    list of vertex indexes forming polytope faces   


---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L167"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L238"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `vertex_enumeration_auctus`

```python
vertex_enumeration_auctus(A, b_max, b_min, b_bias=None)
```

Efficient vertex enumeration algorithm for a problem of a form: Ax = b s.t. b_min <= b <= b_max  

Optional (if b_bias added):  Ax = b s.t. b_min <= b - b_bias <= b_max 

On-line force capability evaluation based on efficient polytope vertex search by A. Skuric, V. Padois, D. Daney 



**Args:**
 
 - <b>`A`</b>:       system matrix A 
 - <b>`b_max`</b>:   maximal b  
 - <b>`b_min`</b>:   minimal b   
 - <b>`b_bias`</b>:  b bias vector ( offset from 0 ) 



**Returns:**
 
 - <b>`f_vertex`</b> (list):   vertices of the polytope 


---

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L339"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L364"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L380"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

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

<a href="https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity/-/blob/master/pycapacity/polyalgos.py#L392"><img align="right" style="float:right;" src="https://img.shields.io/badge/-source-cccccc?style=flat-square"></a>

## <kbd>function</kbd> `stack`

```python
stack(A, B, dir='v')
```








---

_This file was automatically generated via [lazydocs](https://github.com/ml-tooling/lazydocs)._
