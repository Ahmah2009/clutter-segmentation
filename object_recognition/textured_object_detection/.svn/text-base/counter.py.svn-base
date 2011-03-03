# -*- coding: utf-8 -*-
import os, sys, shutil, collections

names = ["100tea", "all", "bp", "coke", "cs", "gdo", "gt", "gtwl", "jp", "naked", "pc", "rrtea", "ts"]

if (len(sys.argv) > 2):
	names = list()	
	f = open(sys.argv[2], 'r')
	for line in f:
		e = line.rfind(' ')
		if (e != -1):
			name = line[0:e]
			names.append(name)
			print name
		


positives = collections.defaultdict(int)
falsePositives = collections.defaultdict(int)
idx = 0
d = dict()
for name in names:
	d[name] = idx
	idx = idx + 1


f = open(sys.argv[1], 'r')
global ojbectName
for line in f:
        #print line,
        s = line.find('test/')
        if s != -1:
                        ind = 5
                        iin = line.find('//')
                        if iin != -1:
                               ind = 6
			e = line.rfind('/')
			name = line[s+ind:e]
			objectName = name
        s = line.find('Object is')
        if s != -1:
			e = line.rfind('is ')
			objectIndex = line[e+3:]
			if int(objectIndex) == d[objectName]:
				positives[objectName] += 1
			else:
				falsePositives[objectName] += 1

for name in names:
	print name, "\t", positives[name], "\t", falsePositives[name]
