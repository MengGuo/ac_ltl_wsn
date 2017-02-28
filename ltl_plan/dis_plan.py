#!/usr/bin/env python

# from mc_model import fts, act, task,


import pickle


# ma_plan = planner.opt_plan()

# r1, g1, r2, g2, r3, g0, r4, g3, r2, g1
ma_plan = [(556, 373), 'g1', (231, 242), 'g2', (844, 138), 'g0', (748, 427), 'g3', (231, 242), 'g1']

ma_path = [((556, 373), 'None'), 
           ((556, 373), 'g1'),
           ((231, 242), 'None'),
           ((231, 242), 'g2'),
           ((844, 138), 'None'),
           ((844, 138), 'g0'),
           ((748, 427), 'None'),
           ((748, 427), 'g3'),
           ((231, 242), 'None'),
           ((231, 242), 'g1'),
]

# prop label: {frozenset(['r4',]): 1.0,}
state_label = {(556,373): 'r1',
               (231,242): 'r2',
               (844,138): 'r3',
               (748,427): 'r4',
}

act = dict()
act['g0'] = (0, 2) # data, duration
act['g1'] = (40, 2) 
act['g2'] = (50, 2) 
act['g3'] = (70, 2) 

pickle.dump([ma_plan, ma_path, act], open('./ma_plan.p','wb'))
pickle.dump([state_label, act], open('./state_action.p','wb'))


