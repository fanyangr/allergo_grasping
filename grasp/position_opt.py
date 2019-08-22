import cvxpy as cp
import numpy as np
def cross_into_matrix(a):
	result = np.zeros((3,3))
	result[0,1] = -a[2]
	result[0,2] = a[1]
	result[1,0] = a[2]
	result[1,2] = -a[0]
	result[2,0] = -a[1]
	result[2,1] = a[0]
	return result
m = 1
g = 9.87
x1 = np.array([0,0,3])
x2 = np.array([2,2,3])
x3 = np.array([2,-2,3])
n1 = np.array([1,0,0])
n2 = np.array([-1,0,0])
n3 = np.array([-1,0,0])
com = np.array([1,0,3])
r1 = x1 - com
r2 = x2 - com 
r3 = x3 - com
r1_m = cross_into_matrix(r1)
r2_m = cross_into_matrix(r2)
r3_m = cross_into_matrix(r3)

f1 = cp.Variable(3)
f2 = cp.Variable(3)
f3 = cp.Variable(3)
gravity = np.zeros(3)
zero = np.zeros(3)

gravity[2] = m * g
constraints = [f1 + f2 + f3 == gravity, r1_m * f1 + r2_m * f2 + r3_m * f3 == zero, f1 * n1 >= 0, f2 * n2 >= 0, f3 * n3 >= 0]

obj = cp.Minimize(cp.norm((f1 - f1 * n1 * n1)) + cp.norm((f2 - f2 * n2 * n2)) + cp.norm((f3 - f3 * n3 * n3)))

prob = cp.Problem(obj, constraints)
prob.solve()
print("status:", prob.status)
print("optimal value", prob.value)
print("optimal var", f1.value, f2.value, f3.value)
