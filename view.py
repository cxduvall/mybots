from solution import SOLUTION
import pickle
import sys
import os
import time

# TODO take cli for run seed, then unpickle and eval PHC from the file

# runSeed viewLen
runSeed = str(int(sys.argv[1]))
viewLen = int(sys.argv[2])

here = path = os.getcwd()
filesHere = os.listdir(here)

filename = None
for i in range(len(filesHere)):
    if filesHere[i].startswith("firstGen_seed" + runSeed):
        filename = filesHere[i]
        break
if filename is None:
    print("NO FIRST FILE FOUND")
    exit()
file = open(filename, 'rb')
parents = pickle.load(file)
print("\nparents:", parents)

print("\nFIRST GEN")
for parent in parents.values():
    parent.Evaluate("view", direct=False, viewLen=viewLen)

print("\nLAST GEN")
time.sleep(1)

filename = None
for i in range(len(filesHere)):
    if filesHere[i].startswith("lastGen_seed" + runSeed):
        filename = filesHere[i]
        break
if filename is None:
    print("NO LAST FILE FOUND")
    exit()
file = open(filename, 'rb')
parents = pickle.load(file)

for parent in parents.values():
    parent.Evaluate("view", direct=False, viewLen=viewLen)
