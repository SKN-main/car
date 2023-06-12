import time
start = time.time()
for i in range(1000000): print(i)
print(time.time()-start)