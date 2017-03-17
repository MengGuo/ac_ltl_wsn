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
Features
-----
* Implementation of the least-squares temporal difference (LSTD) method of the actor-critic type. [[ref1]](http://epubs.siam.org/doi/pdf/10.1137/S0363012901385691) [[ref2]](https://arxiv.org/pdf/1202.2185.pdf)


----
Dependence
----
* Install python packages like Networkx, ply
* Compile [ltl2ba executable](http://www.lsv.ens-cachan.fr/%7Egastin/ltl2ba/download.php) for your OS.
* Compile [ltl2dstar executable](http://www.ltl2dstar.de) for your OS. 
* [Gurobi solver](http://www.gurobi.com) for linear programs. Free for academic use. 
