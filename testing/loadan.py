import pickle

def load_obj(name ):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)


D = load_obj("D")

for i in range(9):

    print i, D[3,2,i]
    print

