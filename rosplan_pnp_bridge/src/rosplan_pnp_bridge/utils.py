# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 16:21:35 2016

@author: cd32
"""

def create_predicate(parameters, *args):
    res = []
    for a in args:
        for e in a:
            t = [str(e.name)]
            t.extend([parameters[str(p.key)] for p in e.typed_parameters])
            res.append("__".join(t))
    return res

def create_condition(operator, *args):
    res = "("+operator
    for a in args:
        a = " ".join(a) if isinstance(a, list) else a
        res += " "+a
    return res+")"