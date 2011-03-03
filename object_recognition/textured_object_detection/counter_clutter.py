# -*- coding: utf-8 -*-
import os, sys, shutil, collections

global _all 
global _gt 
global _gdo

_all = 0
_gt = 0
_gdo = 0
_fp = 0

f = open(sys.argv[1], 'r')
for line in f:
        s = line.find('Object\'s id')
        if s != -1:
			e = line.rfind('id - ')
			objectIndex = line[e+5:e+6]
			if int(objectIndex) == 1:
				_all += 1
			elif int(objectIndex) == 5:
				_gdo += 1
			elif int(objectIndex) == 6:
				_gt += 1
			else:
				_fp += 1

print _all, "\t", _gdo, "\t", _gt, "\t", _fp
