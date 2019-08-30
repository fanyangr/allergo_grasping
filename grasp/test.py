import cvxpy as cp
import numpy as np
import redis
import json

PYTHON_START_FLAG_KEY = "start_flag_key"
# CONTACT_POSITION_KEYS = ["contact_position_1_key", "contact_position_2_key", "contact_position_3_key"]
# CONTACT_NORMAL_KEYS = ["contact_normal_1_key","contact_normal_2_key","contact_normal_3_key"]
CONTACT_POSITION_1_KEY = "contact_position_1_key"
CONTACT_POSITION_2_KEY = "contact_position_2_key"
CONTACT_POSITION_3_KEY = "contact_position_3_key"
CONTACT_NORMAL_1_KEY = "contact_normal_1_key"
CONTACT_NORMAL_2_KEY = "contact_normal_2_key"
CONTACT_NORMAL_3_KEY = "contact_normal_3_key"
FORCE_1_KEY = "force_1_key"
FORCE_2_KEY = "force_2_key"
FORCE_3_KEY = "force_3_key"
MASS_KEY = "mass_key"
COM_KEY = "com_key"
SCORE_KEY = "score_key"
coefficient_of_friction = 0.5


def cross_into_matrix(a):
	result = np.zeros((3,3))
	result[0,1] = -a[2]
	result[0,2] = a[1]
	result[1,0] = a[2]
	result[1,2] = -a[0]
	result[2,0] = -a[1]
	result[2,1] = a[0]
	return result

def optimize():
	# values needed to be recived from the redis
	start_flag = json.loads(server.get(PYTHON_START_FLAG_KEY).decode("utf-8"))
	# start_flag = True
	g = 9.87
	if start_flag == True:
		# values needed to be received from redis
		# x1 = json.loads(server.get(CONTACT_POSITION_1_KEY).decode("utf-8"))
		# x2 = json.loads(server.get(CONTACT_POSITION_2_KEY).decode("utf-8"))
		# x3 = json.loads(server.get(CONTACT_POSITION_3_KEY).decode("utf-8"))
		# n1 = json.loads(server.get(CONTACT_NORMAL_1_KEY).decode("utf-8"))		
		# n2 = json.loads(server.get(CONTACT_NORMAL_2_KEY).decode("utf-8"))
		# n3 = json.loads(server.get(CONTACT_NORMAL_3_KEY).decode("utf-8"))
		# m = json.loads(server.get(MASS_KEY).decode("utf-8"))
		# com = json.loads(server.get(COM_KEY).decode("utf-8"))
		# x1 = np.array(x1)
		# x2 = np.array(x2)
		# x3 = np.array(x3)
		# n1 = np.array(n1)
		# n2 = np.array(n2)
		# n3 = np.array(n3)
		# com = np.array(com)
		# m = float(m)
		iter_coefficient_of_friction = coefficient_of_friction


		m = 1
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

		f1_lateral = f1 - f1 * n1 * n1
		f2_lateral = f2 - f2 * n2 * n2
		f3_lateral = f3 - f3 * n3 * n3
		gravity = np.zeros(3)
		zero = np.zeros(3)

		gravity[2] = m * g
		iter_counter = 0
		while iter_counter < 8:
			constraints = [f1 + f2 + f3 == gravity, r1_m * f1 + r2_m * f2 + r3_m * f3 == zero, f1 * n1 >= 0, f2 * n2 >= 0, f3 * n3 >= 0\
			, cp.norm(f1_lateral) <= iter_coefficient_of_friction * f1 * n1, cp.norm(f2_lateral) <= iter_coefficient_of_friction * f2 * n2, cp.norm(f3_lateral) <= iter_coefficient_of_friction * f3 * n3]
			# obj = cp.Minimize(1/cp.norm(f1))
			obj = cp.Minimize((cp.norm((f1 - f1 * n1 * n1))) + cp.norm((f2 - f2 * n2 * n2)) + cp.norm((f3 - f3 * n3 * n3))\
				+ 0.1 * (cp.norm(f1) + cp.norm(f2) + cp.norm(f3)))

			prob = cp.Problem(obj, constraints)
			print("This is the contact points for the thumb: ")
			print(x1)
			prob.solve()
			if prob.status in ["infeasible", "unbounded"]:
				iter_coefficient_of_friction *= 1.2
				iter_counter += 1 
				print("fail. try with mu: ", iter_coefficient_of_friction)
			else: 
				break
		print("status:", prob.status)
		print("optimal value", prob.value)
		print("optimal var", f1.value, f2.value, f3.value)
		server.set(SCORE_KEY, json.dumps(prob.value))
		server.set(PYTHON_START_FLAG_KEY, json.dumps(0))
		server.set(FORCE_1_KEY, json.dumps(list(f1.value)))
		server.set(FORCE_2_KEY, json.dumps(list(f2.value)))
		server.set(FORCE_3_KEY, json.dumps(list(f3.value)))
		

if __name__ == "__main__":
	print("The python code for the optimization is running!")
	server = redis.Redis()
	server.set(PYTHON_START_FLAG_KEY, json.dumps(True))
	while True:
		optimize()