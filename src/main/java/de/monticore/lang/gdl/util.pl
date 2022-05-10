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
