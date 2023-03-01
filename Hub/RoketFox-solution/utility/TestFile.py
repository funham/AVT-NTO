import numpy as np

l = 4
ids = list(range(l))
for i in range(l):
            print(ids) 
            ids.append(ids[0])
            del(ids[0])


# 0123 0
# 1230 1
# 2301 2
# 3012 3
