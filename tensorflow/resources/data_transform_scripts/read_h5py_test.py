import h5py

f = h5py.File('train.h5', 'r')

l = list(f.keys())

dset = f['data']
lable = f['target_label']

print dset[0]
print lable[0]
#print lable.shape






#

