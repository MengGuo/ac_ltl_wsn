#!/usr/bin/env python

import pickle



def load_plan(plan_file_dir='./ltl_plan/ma_plan.p'):
    ma_plan, ma_path, act = pickle.load(open(plan_file_dir,'rb'))
    print '------------------------------'
    print 'motion and action plan: %s' %ma_plan
    print 'motion and action path: %s' %ma_path
    return ma_plan, ma_path, act


def find_cri_segs(ma_plan, ma_path, act, data_bound):    
    cri_segs = []
    cri_segs_index = []
    buffer_before = 0
    ind_before = 0
    buffer_after = 0
    ind_after = 0
    for i, todo in enumerate(ma_plan):
        if isinstance(todo, basestring):
            # action
            buffer_after = buffer_before + act[todo][0]
            if 0 <= buffer_after <= data_bound:
                buffer_before = buffer_after
                ind_before = i
            else:
                ind_after = i-1
                cri_segs.append(tuple((ma_path[ind_before], ma_path[ind_after])))
                cri_segs_index.append((ind_before, ind_after))
                buffer_before = act[todo][0]
                buffer_after = act[todo][0]
                ind_before = i
    cri_segs.append((ma_path[ind_before], ma_path[0]))
    cri_segs_index.append((ind_before, 0))
    print '--- Find **%d** critical segments: %s with index %s ---' %(len(cri_segs), str(cri_segs), str(cri_segs_index))
    return cri_segs, cri_segs_index




    
