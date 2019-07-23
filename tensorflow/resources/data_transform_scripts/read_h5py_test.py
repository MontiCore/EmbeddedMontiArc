import h5py

f = h5py.File('test.h5', 'r')

l = list(f.keys())

dset = f['data']
lable = f['ergebnis_label']

print dset[0]
#print lable.shape





#

