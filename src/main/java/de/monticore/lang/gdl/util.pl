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
