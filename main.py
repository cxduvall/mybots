import os
import sys

if len(sys.argv) > 1:
    os.system("python3 search.py " + sys.argv[1])
else:
    os.system("python3 search.py")