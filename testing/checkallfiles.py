import os

l = []

d = './a0'
folders = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]


for f in folders:

    x = f[5:].split("_")
    test_var = int(x[0])
    variation = int(x[1])
    starting_dist = int(x[2])
    deviation = int(x[3])
    random_seed = int(x[4])


    l.append((test_var,variation,starting_dist,deviation,random_seed))



d = './a1'
folders = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]


for f in folders:

    x = f[5:].split("_")
    test_var = int(x[0])
    variation = int(x[1])
    starting_dist = int(x[2])
    deviation = int(x[3])
    random_seed = int(x[4])


    l.append((test_var,variation,starting_dist,deviation,random_seed))


d = './a2'
folders = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]


for f in folders:

    x = f[5:].split("_")
    test_var = int(x[0])
    variation = int(x[1])
    starting_dist = int(x[2])
    deviation = int(x[3])
    random_seed = int(x[4])


    l.append((test_var,variation,starting_dist,deviation,random_seed))


d = './a3'
folders = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]


for f in folders:
    

    x = f[5:].split("_")
    test_var = int(x[0])
    variation = int(x[1])
    starting_dist = int(x[2])
    deviation = int(x[3])
    random_seed = int(x[4])


    l.append((test_var,variation,starting_dist,deviation,random_seed))


for a in range(4):
    for b in range(3):
        for c in range(80):
            for d in range(7):
                for e in range(1):
                    if (a,b,c,d,e) in l:
                        print a,b,c,d,e


