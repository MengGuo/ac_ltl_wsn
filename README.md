ac_ltl_wsn
========

Temporal Motion and Communicaiton Planning in Wirelessly Connected Environments via Actor-critic Reinforcement Learning 

-----
Description
-----
This package contains the implementation for motion and communication control of a mobile robot, which is tasked with gathering data in an environment of interest and transmitting these data to a data center.
The task is specified as a high-level Linear Temporal Logic (LTL) formula that captures the data to be gathered at various regions in the workspace. 
The robot has a limited buffer to store the data, which needs to be transmitted to the data center before the buffer overflows. Communication between the robot and the data center is through a dedicated wireless network to which the robot can upload data with rates that are uncertain and unknown. 


<p align="center">  
  <img src="https://github.com/MengGuo/ac_ltl_wsn/blob/master/figures/frame75.png" width="700"/>
</p>



-----
Content
-----
* Sampling-based roadmap construction, see [[build_roadmap]](https://github.com/MengGuo/ac_ltl_wsn/tree/master/build_roadmap) folder

<p align="center">  
  <img src="https://github.com/MengGuo/ac_ltl_wsn/blob/master/build_roadmap/roadmap_2D.png" width="700"/>
</p>

* Wireless routing based on Linear Programming (LP), see [wsn_routing](https://github.com/MengGuo/ac_ltl_wsn/tree/master/wsn_routing) folder

<p align="center">  
  <img src="https://github.com/MengGuo/ac_ltl_wsn/blob/master/wsn_routing/wsn_rate.png" width="700"/>
</p>

* Policy synthesis given a fully-known system model as Markov Decision Processes (MDP)
  * Product automaton between MDP and Deterministic Robin Automaton (DRA), based on [[MDP_TG]](https://github.com/MengGuo/ac_ltl_wsn/tree/master/product_based/MDP_TG).

  * Policy generated via LP, see [[lp_policy.p]](https://github.com/MengGuo/ac_ltl_wsn/blob/master/product_based/lp_policy.py)

  * Policy generated via actor-critic RL in the product, see [[ac_policy.p]](https://github.com/MengGuo/ac_ltl_wsn/blob/master/product_based/ac_policy.py)

  * Example log, see [[log.txt]](https://github.com/MengGuo/ac_ltl_wsn/blob/master/product_based/log.txt)

* Implementation of the least-squares temporal difference (LSTD) method of the actor-critic type. [[ref1]](http://epubs.siam.org/doi/pdf/10.1137/S0363012901385691) [[ref2]](https://arxiv.org/pdf/1202.2185.pdf)

  * Task execution, workspace exploration, parameterized-policy learning all performed online and simultaneously, see [[ac.py]] (https://github.com/MengGuo/ac_ltl_wsn/blob/master/ac.py)
  * For single critical segment, see [[one_cri_seg_ac_learn.py]](https://github.com/MengGuo/ac_ltl_wsn/blob/master/one_cri_seg_ac_learn.py)
  * For a given high-level discrete plan, see [[ltl_ac_learn.py]](https://github.com/MengGuo/ac_ltl_wsn/blob/master/ltl_ac_learn.py)
  * **Indirect** learning mode via simulated experience, and **direct** learning via real experience. 

```python
from crm import build_crm 
from ac import actor_critic

# load roadmap with wsn rate info, as combined road map (crm)
crm = build_crm()

# set up actor_critic learner
actor_critic_learner = actor_critic(crm, data_bound, quant_size,
		       		Ts, uncertainty_prob, clambda,
				gamma, beta, D)
actor_critic_learner.set_init_goal(new_init, new_goal)                
actor_critic_learner.set_theta(Theta[cri_seg])

# indrect learn
print '|||||||Indirect learning for %d episodes|||||||' %static_learn_episodes
indirect_learn_log = actor_critic_learner.complete_learn(static_learn_episodes, mode ='model')

# robot start moving
print '|||||||Direct learning for 1 episode|||||||'
direct_learn_log = actor_critic_learner.one_episode_learn(gamma_seq[k], beta_seq[k], mode='experiment')
```

<p align="center">  
  <img src="https://github.com/MengGuo/ac_ltl_wsn/blob/master/figures/ltl_ac_theta.png" width="700"/>
</p>

----
Dependence
----
* Install python packages like Networkx, ply
* Compile [ltl2ba executable](http://www.lsv.ens-cachan.fr/%7Egastin/ltl2ba/download.php) for your OS.
* Compile [ltl2dstar executable](http://www.ltl2dstar.de) for your OS. 
* [Gurobi solver](http://www.gurobi.com) for linear programs. Free for academic use. 
