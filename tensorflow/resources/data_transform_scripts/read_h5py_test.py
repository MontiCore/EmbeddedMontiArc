import h5py

f = h5py.File('train.h5', 'r')

l = list(f.keys())

dset = f['data']
lable = f['ergebnis_lable']

print dset.shape
print lable.shape





#

