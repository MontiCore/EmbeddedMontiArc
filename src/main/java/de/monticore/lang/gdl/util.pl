:- use_module(library(random)).
get_random_legal(A) :-
  setof((X), function_legal(value_random, X), Models),
  random_member(A, Models).

% positive
atom_to_number(Atom, Number) :-
  atom_concat(value_, OnlyNumber, Atom),
  atom_number(OnlyNumber, Number).

% negative
atom_to_number(Atom, Number) :-
  atom_concat(valnn_, OnlyNumber, Atom),
  atom_concat('-', OnlyNumber, SignedNumberAtom),
  atom_number(SignedNumberAtom, Number).


% positive
number_to_atom(Number, Atom) :-
  (1 is sign(Number); 0 is sign(Number)),
  atom_number(OnlyNumber, Number),
  atom_concat(value_, OnlyNumber, Atom).

% negative
number_to_atom(Number, Atom) :-
  -1 is sign(Number),
  PNumber is -1 * Number,
  atom_number(OnlyNumber, PNumber),
  atom_concat(valnn_, OnlyNumber, Atom).

:- dynamic(state/1).
:- dynamic(state/2).
:- dynamic(state_hidden/2).
:- dynamic(state_hidden/3).

:- dynamic(input/1).

retractAllStates() :-
  retractall(state(X0)),
  retractall(state(X1, Y1)),
  retractall(state_hidden(X2, Y2)),
  retractall(state_hidden(X3, Y3, Z3)).

% -----Addition-----
% Z is variable or no Variables
add(X, Y, Z) :-
  nonvar(X),
  nonvar(Y),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NZ is NX + NY,
  number_to_atom(NZ, Z).

% Y is variable
add(X, Y, Z) :-
  nonvar(X),
  nonvar(Z),
  atom_to_number(X, NX),
  atom_to_number(Z, NZ),
  NY is NZ - NX,
  number_to_atom(NY, Y).

% X is variable
add(X,Y,Z) :-
  nonvar(Y),
  nonvar(Z),
  atom_to_number(Y, NY),
  atom_to_number(Z, NZ),
  NX is NZ - NY,
  number_to_atom(NX, X).

% -----Subtraction-----
% Z is variable or no Variables
sub(X, Y, Z) :-
  nonvar(X),
  nonvar(Y),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NZ is NX - NY,
  number_to_atom(NZ, Z).

% Y is variable
sub(X, Y, Z) :-
  nonvar(X),
  nonvar(Z),
  atom_to_number(X, NX),
  atom_to_number(Z, NZ),
  NY is NX - NZ,
  number_to_atom(NY, Y).

% X is variable
sub(X, Y, Z) :-
  nonvar(Y),
  nonvar(Z),
  atom_to_number(Y, NY),
  atom_to_number(Z, NZ),
  NX is NY + NZ,
  number_to_atom(NX, X).

% -----Multiplication-----
% Z is variable or no Variables
mult(X, Y, Z) :-
  nonvar(X),
  nonvar(Y),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NZ is NX * NY,
  number_to_atom(NZ, Z).

% Y is variable
mult(X, Y, Z) :-
  nonvar(X),
  nonvar(Z),
  atom_to_number(X, NX),
  atom_to_number(Z, NZ),
  NX \== 0,
  NY is NZ / NX,
  NYF is round(NY),
  number_to_atom(NYF, Y).

% X is variable
mult(X, Y, Z) :-
  nonvar(Y),
  nonvar(Z),
  atom_to_number(Z, NZ),
  atom_to_number(Y, NY),
  NY \== 0,
  NX is NZ / NY,
  NXF is round(NX),
  number_to_atom(NXF, X).

% -----Division-----
% Z is variable or no Variables
div(X, Y, Z) :-
  nonvar(X),
  nonvar(Y),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NY \== 0,
  NZ is NX / NY,
  NZF is round(NZ),
  number_to_atom(NZF, Z).

% Y is variable
div(X, Y, Z) :-
  nonvar(X),
  nonvar(Z),
  atom_to_number(X, NX),
  atom_to_number(Z, NZ),
  NZ \== 0,
  NY is NX / NZ,
  NYF is round(NY),
  number_to_atom(NYF, Y).

% X is variable
div(X, Y, Z) :-
  nonvar(Y),
  nonvar(Z),
  atom_to_number(Z, NZ),
  atom_to_number(Y, NY),
  NX is NY * NZ,
  number_to_atom(NX, X).

% -----Successor Relation-----
% Y is Variable
succ(X,Y) :-
  nonvar(X),
  atom_to_number(X, NX),
  NY is NX + 1,
  number_to_atom(NY, Y).

% X is Variable
succ(X,Y) :-
  nonvar(Y),
  atom_to_number(Y, NY),
  NX is NY - 1,
  number_to_atom(NX, X).

% -----Value Comparision-----
less(X,Y) :-
  nonvar(X),
  nonvar(Y),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NX < NY.

greater(X,Y) :-
  less(Y,X).

equal(X,Y) :-
  nonvar(X),
  nonvar(Y),
  X = Y.

% ----- is a number? -----
num(X) :-
  nonvar(X),
  atom_to_number(X, NX),
  integer(NX).

% ----- Modulo -----
modulo(X, Y, Z) :-
  nonvar(X),
  nonvar(Y),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NY \== 0,
  NZ is NX mod NY,
  number_to_atom(NZ, Z).
