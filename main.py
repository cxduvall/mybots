import os
import sys

args = " ".join(sys.argv[1:])
if len(sys.argv) > 1:
    os.system("python3 search.py " + args)
else:
    os.system("python3 search.py")