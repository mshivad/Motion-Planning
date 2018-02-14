
#!/usr/bin/env python

import collisions
import rrt
import prm


#rrt.test_rrt_env(num_samples=500, step_length=2, env='./env0.txt', connect= False)
#rrt.test_rrt_env(num_samples=500, step_length=2, env='./env0.txt', connect= 1)
#rrt.test_rrt_env(num_samples=500, step_length=2, env='./env0.txt', connect= 2)
#rrt.test_rrt_env(num_samples=5000, step_length=0.18, env='./env1.txt', connect= False)
#rrt.test_rrt_env(num_samples=5000, step_length=0.18, env='./env1.txt', connect= 1)
#rrt.test_rrt_env(num_samples=5000, step_length=0.18, env='./env1.txt', connect= 2)


#prm.test_prm_env(num_samples=200, step_length=5, env='./env0.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=1000, step_length=5, env='./env0.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=1200, step_length=5, env='./env0.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=800, step_length=5, env='./env0.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=800, step_length=5, env='./env0.txt', localrad=20, variance=10, no_closest_neighbours= 10, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=800, step_length=5, env='./env0.txt', localrad=20, variance=10, no_closest_neighbours= 10, gaussian=False, rrtlp=False)

prm.test_prm_env(num_samples=300, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 10, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=300, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=1200, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=800, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=800, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 10, gaussian=False, rrtlp=False)
#prm.test_prm_env(num_samples=800, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 10, gaussian=False, rrtlp=False)

#prm.test_prm_env(num_samples=800, step_length=5, env='./env0.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=True, rrtlp=False)
#prm.test_prm_env(num_samples=800, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=True, rrtlp=False)

#prm.test_prm_env(num_samples=800, step_length=5, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=True)
#prm.test_prm_env(num_samples=800, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=False, rrtlp=True)
#prm.test_prm_env(num_samples=1200, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=True, rrtlp=True)
#prm.test_prm_env(num_samples=800, step_length=0.1, env='./env1.txt', localrad=20, variance=10, no_closest_neighbours= 5, gaussian=True, rrtlp=True)