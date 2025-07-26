%herm(m1,m2,op).
herm(X,Y,' + '):- herm(X), herm(Y).
herm(X,Y,' - '):- herm(X), herm(Y).
herm(X,Y,' * '):- herm(X), scal(Y).
herm(X,Y,' * '):- herm(X), int(Y).
herm(X,Y,' * '):- herm(Y), scal(X).
herm(X,Y,' * '):- herm(Y), int(X).
herm(X,Y,' ^ '):- herm(X), int(Y).
herm(X,'inv'):- herm(X), inv(X).
herm(X,'trans'):- herm(X).

%diag(m1,m2,op).
diag(X,Y,' + '):- diag(X), diag(Y).
diag(X,Y,' - '):- diag(X), diag(Y).
diag(X,Y,' * '):- diag(X), diag(Y).
diag(X,Y,' * '):- diag(X), scal(Y).
diag(X,Y,' * '):- diag(X), int(Y).
diag(X,Y,' * '):- diag(Y), scal(X).
diag(X,Y,' * '):- diag(Y), int(X).
diag(X,Y,' ^ '):- diag(X), int(Y).
diag(X,'inv'):- diag(X), inv(X).
diag(X,'trans'):- diag(X).

%psd(m1,m2,op).
psd(X,Y,' + '):- psd(X), psd(Y).
psd(X,Y,' - '):- psd(X), nsd(Y).
psd(X,Y,' * '):- psd(X), scal+(Y).
psd(X,Y,' * '):- psd(X), int+(Y).
psd(X,Y,' * '):- psd(Y), scal+(X).
psd(X,Y,' * '):- psd(Y), int+(X).
psd(X,Y,' + '):- diag(X), pos(X), diag(Y), pos(Y).
psd(X,Y,' - '):- diag(X), pos(X), diag(Y), neg(Y).
psd(X,Y,' * '):- diag(X), pos(X), diag(Y), pos(Y).
psd(X,Y,' * '):- diag(X), neg(X), diag(Y), neg(Y).

%pd(m1,m2,op).
pd(X,Y,' + '):- pd(X), pd(Y).
pd(X,Y,' - '):- pd(X), nsd(Y).
pd(X,Y,' * '):- pd(X), scal+(Y).
pd(X,Y,' * '):- pd(X), int+(Y).
pd(X,Y,' * '):- pd(Y), scal+(X).
pd(X,Y,' * '):- pd(Y), int+(X).

%nsd(m1,m2,op).
nsd(X,Y,' + '):- nsd(X), nsd(Y).
nsd(X,Y,' - '):- nsd(X), psd(Y).
nsd(X,Y,' * '):- nsd(X), scal+(Y).
nsd(X,Y,' * '):- nsd(X), int+(Y).
nsd(X,Y,' * '):- nsd(Y), scal+(X).
nsd(X,Y,' * '):- nsd(Y), int+(X).
nsd(X,Y,' + '):- diag(X), neg(X), diag(Y), neg(Y).
nsd(X,Y,' - '):- diag(X), neg(X), diag(Y), pos(Y).
nsd(X,Y,' * '):- diag(X), neg(X), diag(Y), pos(Y).
nsd(X,Y,' * '):- diag(X), pos(X), diag(Y), neg(Y).

%nd(m1,m2,op).
nd(X,Y,' + '):- nd(X), nd(Y).
nd(X,Y,' - '):- nd(X), psd(Y).
nd(X,Y,' * '):- nd(X), scal+(Y).
nd(X,Y,' * '):- nd(X), int+(Y).
nd(X,Y,' * '):- nd(Y), scal+(X).
nd(X,Y,' * '):- nd(Y), int+(X).

%norm(m1,m2, * ).
norm(X,Y,' * '):- norm(X), scal(Y).
norm(X,Y,' * '):- norm(X), int(Y).
norm(X,Y,' * '):- norm(Y), scal(X).
norm(X,Y,' * '):- norm(Y), int(X).
norm(X,Y,' ^ '):- norm(X), int(Y).
norm(X,Y,' + '):- herm(X), herm(Y).
norm(X,Y,' - '):- herm(X), herm(Y).
norm(X,'inv'):- herm(X), inv(X).
norm(X,'trans'):- herm(X).

%skewHerm(m1,m2,op).
skewHerm(X,Y,' + '):- skewHerm(X), skewHerm(Y).
skewHerm(X,Y,' - '):- skewHerm(X), skewHerm(Y).
skewHerm(X,Y,' * '):- skewHerm(X), scal(Y).
skewHerm(X,Y,' * '):- skewHerm(X), int(Y).
skewHerm(X,Y,' * '):- skewHerm(Y), scal(X).
skewHerm(X,Y,' * '):- skewHerm(Y), int(X).
skewHerm(X,'inv'):- skewHerm(X), inv(X).
skewHerm(X,'trans'):- skewHerm(X).

%square(m1,m2,op).
square(X,Y,' + '):- square(X), square(Y).
square(X,Y,' - '):- square(X), square(Y).
square(X,Y,' * '):- square(X), square(Y).
square(X,Y,' * '):- square(X), scal(Y).
square(X,Y,' * '):- square(X), int(Y).
square(X,Y,' * '):- square(Y), scal(X).
square(X,Y,' * '):- square(Y), int(X).
square(X,'inv'):- square(X), inv(X).
square(X,Y,' ^ '):- square(X), int(Y).
square(X,'trans'):- square(X).

%inv(m1,m2,op).
inv(X,Y,' + '):- diag(X), pos(X), diag(Y), pos(Y).
inv(X,Y,' + '):- diag(X), neg(X), diag(Y), neg(Y).
inv(X,Y,' - '):- diag(X), pos(X), diag(Y), neg(Y).
inv(X,Y,' - '):- diag(X), neg(X), diag(Y), pos(Y).
inv(X,Y,' * '):- diag(X), pos(X), diag(Y), pos(Y).
inv(X,Y,' * '):- diag(X), pos(X), diag(Y), neg(Y).
inv(X,Y,' * '):- diag(X), neg(X), diag(Y), pos(Y).
inv(X,Y,' * '):- diag(X), neg(X), diag(Y), neg(Y).
inv(X,'inv'):- square(X), inv(X).

%pos(m1,m2,op).
pos(X,Y,' + '):- pos(X), pos(Y).
pos(X,Y,' - '):- pos(X), neg(Y).

%pos(m1,m2,op).
neg(X,Y,' + '):- neg(X), neg(Y).
neg(X,Y,' - '):- neg(X), neg(Y).

%conclusions
square(X):- norm(X).
norm(X):- diag(X); herm(X); skewHerm(X).
herm(X):- pd(X); psd(X); nsd(X); nd(X); indef(X).
psd(X):- pd(X).
nsd(X):- nd(X).
