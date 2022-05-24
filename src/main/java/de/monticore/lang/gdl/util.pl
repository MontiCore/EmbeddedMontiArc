:- use_module(library(random)).
get_random_legal(A) :-
  setof((X), function_legal(value_random, X), Models),
  random_member(A, Models).

atom_to_number(Atom, Number) :-
  atom_concat(value_, OnlyNumber, Atom),
  atom_number(OnlyNumber, Number).

number_to_atom(Number, Atom) :-
  atom_number(OnlyNumber, Number),
  atom_concat(value_, OnlyNumber, Atom).

:- dynamic(state/1).
:- dynamic(state/2).
:- dynamic(state_hidden/2).
:- dynamic(state_hidden/3).

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
  substract(Z, X, Y).

% X is variable
add(X,Y,Z) :-
  nonvar(Y),
  nonvar(Z),
  substract(Z, Y, X).

% -----Subtraction-----
% Z is variable or no Variables
substract(X, Y, Z) :-
  nonvar(X),
  nonvar(Y),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NZ is NX - NY,
  number_to_atom(NZ, Z).

% Y is variable
substract(X, Y, Z) :-
  nonvar(X),
  nonvar(Z),
  substract(X, Z, Y).

% X is variable
substract(X, Y, Z) :-
  nonvar(Y),
  nonvar(Z),
  add(Y, Z, X).

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
  div(Z,X,Y).

% X is variable
mult(X, Y, Z) :-
  nonvar(Y),
  nonvar(Z),
  div(Z,Y,X).

% -----Division-----
% Z is variable or no Variables
div(X, Y, Z) :-
  nonvar(Y),
  nonvar(Z),
  atom_to_number(X, NX),
  atom_to_number(Y, NY),
  NZ is NX / NY,
  number_to_atom(NZ, Z).

% Y is variable
div(X, Y, Z) :-
  nonvar(X),
  nonvar(Z),
  div(X,Z,Y).

% X is variable
div(X, Y, Z) :-
  nonvar(Y),
  nonvar(Z),
  mult(Y,Z,X).

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