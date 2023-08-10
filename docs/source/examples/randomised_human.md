Randomized Human musculoskeletal models
=======================================

A simple example program for force polytope evaluation of a randomised human musculoskeletal model. 
Simply change the number of dof, number of forces and force limits and see how the calculation time and shape evaluates.


<div class="admonition-new-examples admonition">
<p class="admonition-title">📢 NEW Examples!</p>
<dl class="simple">
<dt>For some more examples check out the <code class="docutils literal notranslate"><span class="pre">examples</span></code> folder of the repository.</dt><dd><ul class="simple">
<li><p>Interactive jupyter notebooks are available in the <code class="docutils literal notranslate"><span class="pre">examples/notebooks</span></code> folder: <a class="reference external" href="https://github.com/auctus-team/pycapacity/blob/master/examples/notebooks/demo_simple.ipynb">see on Github</a></p></li>
</ul>
</dd>
</dl>
</div>


```python
import pycapacity.human as capacity # robot capacity module
import numpy as np

L = 20 # number of muscles
m = 3 # 3d forces
n = 6 # number of joints - dof

# this seed is used to generate the same image 
# as in the examples in the docs 
np.random.seed(123456)

J = np.array(np.random.rand(m,n))*2-1 # random jacobian matrix
N = np.array(np.random.rand(n,L))*2-1 # random moment arm matrix

F_max = 100*np.ones(L)  # muscle forces limits max and min
F_min = np.zeros(L)

f_poly = capacity.force_polytope(J,N, F_min, F_max, 0.1) # calculate the polytope 

print(f_poly.vertices) # display the vertices


# plotting the polytope
import matplotlib.pyplot as plt
from pycapacity.visual import * # pycapacity visualisation tools
fig = plt.figure(4)

# draw faces and vertices
plot_polytope(plot=plt, 
              polytope=f_poly, 
              label='force', 
              edge_color='black', 
              alpha = 0.4)

plt.legend()
plt.show()

```
![](../images/rand_human.png)
