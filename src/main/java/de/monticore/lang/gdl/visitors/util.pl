:- use_module(library(random)).
:- use_module(library(aggregate)).
:- style_check(-singleton).
:- style_check(-discontiguous).
:- set_prolog_flag(answer_write_options,[max_depth(0)]).

:- dynamic(gdli_input/1).
:- dynamic(gdl_state/1).
:- dynamic(gdl_state_hidden/2).
:- dynamic(gdli_options/1).

gdl_init(X) :- false.
gdl_init_hidden(X, Y) :- false.

gdl_next(X) :- false.
gdl_next_hidden(X, Y) :- false.

gdli_input(X) :- false.

gdli_options(X) :- false.

gdli_with_types() :- false.

% ----------------------------
% ------- Interpreter --------
% ----------------------------

gdli_random_legal(Move) :-
    setof(X, gdl_legal(value_random, X), Models),
    random_member(B, Models),
    Move = [value_random, B].

gdli_do_random() :-
    \+ gdli_options(manual_random),
    gdl_role(value_random),
    gdli_random_legal(Move),
    gdli_do_move(Move).
gdli_do_random().

gdli_retract_state() :-
    retractall(gdl_state(X)),
    retractall(gdl_state_hidden(Y, Z)).


gdli_init_state(State) :-
    setof(X, gdl_init(X), State),
    !.
gdli_init_state([]).

gdli_init_hidden_state(HiddenState) :-
    setof([X, Y], gdl_init_hidden(X, Y), HiddenState),
    !.
gdli_init_hidden_state([]).


gdli_next_state(State) :-
    setof(X, gdl_next(X), State),
    !.
gdli_next_state([]).

gdli_next_hidden_state(HiddenState) :-
    setof([X, Y], gdl_next_hidden(X, Y), HiddenState),
    !.
gdli_next_hidden_state([]).



gdli_init() :-
    gdli_init_templates(),
    gdli_init_state(State),
    gdli_init_hidden_state(HiddenState),
    gdli_load_state(State),
    gdli_load_hidden_state(HiddenState),
    gdli_do_random(),
    !.


gdli_init_templates() :-
    \+ gdli_with_types(),
    !.
gdli_init_templates() :-
    gdli_with_types(),
    \+ gdlt_all_templates_merged(_, _),
    gdlt_make_all_templates(),
    !.
gdli_init_templates().

gdli_build_next() :-
    gdli_next_state(State),
    gdli_next_hidden_state(HiddenState),
    gdli_retract_state(),
    gdli_load_state(State),
    gdli_load_hidden_state(HiddenState),
    !.


gdli_load_state([]).
gdli_load_state([X]) :- 
    assertz(gdl_state(X)).
gdli_load_state([X|Xs]) :-
    gdli_load_state([X]),
    gdli_load_state(Xs).

gdli_load_hidden_state([]).
gdli_load_hidden_state([[X, Y]]) :- 
    assertz(gdl_state_hidden(X, Y)).
gdli_load_hidden_state([X|Xs]) :-
    gdli_load_hidden_state([X]),
    gdli_load_hidden_state(Xs).


gdli_reset() :-
    gdli_retract_state(),
    gdli_init(),
    !.


gdli_is_legal([Role, Move]) :-
    gdl_legal(Role, Move),
    !.

gdli_do_move([Player, [value_noop]]) :-
    gdli_is_legal([Player, [value_noop]]),
    !.
gdli_do_move(Move) :-
    gdli_is_legal(Move),
    gdli_assert_move(Move),
    gdli_build_next(),
    gdli_retract_move(),
    gdli_do_random(),
    !.

gdli_assert_move(Move) :-
    assertz(gdli_input(Move)).

gdli_retract_move() :-
    retractall(gdli_input(X)).


gdli_all_legal_moves(Models) :-
    setof([Role, Move], gdl_legal(Role, Move), Models),
    !.
gdli_all_legal_moves([]).

gdli_all_legal_moves(Role, Models) :-
    setof([Role, Move], gdl_legal(Role, Move), Models),
    !.
gdli_all_legal_moves(_, []).


gdli_all_state(Models) :-
    setof(X, gdl_state(X), Models),
    !.
gdli_all_state([]).

gdli_all_state_hidden(Models) :-
    setof([X, Y], gdl_state_hidden(X, Y), Models),
    !.
gdli_all_state_hidden([]).

gdli_all_state_hidden_role(Role, Models) :-
    setof(X, gdl_state_hidden(Role, X), Models),
    !.
gdli_all_state_hidden_role(_, []).

gdli_full_state_role(Role, Models) :-
    gdli_all_state(OpenModels),
    gdli_all_state_hidden_role(Role, HiddenModels),
    append(OpenModels, HiddenModels, Models),
    !.

gdli_all_goal(Models) :-
    setof([X, Y], gdli_goal(X, Y), Models),
    !.
gdli_all_goal([]).

gdli_all_role(Models) :-
    setof(X, gdl_role(X), Models),
    !.
gdli_all_role([]).

% ----------------------------
% -------- Functions ---------
% ----------------------------

gdli_not(X) :-
    \+ gdl_rule(X).

gdli_distinct([]).
gdli_distinct(X) :-
    msort(X, Sorted),
    gdli_sorted_distinct(Sorted).

gdli_sorted_distinct([]).
gdli_sorted_distinct([_]).
gdli_sorted_distinct([X,Y|Xs]) :-
    X \== Y,
    gdli_sorted_distinct([Y|Xs]).

gdl_count(N, Goal) :-
    aggregate_all(count, Goal, Count),
    Count > 0,
    gdli_number_to_atom(Count, N).

gdl_legal(Role, Move) :-
    gdl_rule([legal, Role, Move]).

gdl_role(X) :-
    gdl_rule([role, X]).

% ----------------------------
% -------- Arithmetic --------
% ----------------------------

% -- Map Numbers --
% -----------------

% Positive
gdli_atom_to_number(Atom, Number) :-
    atom_concat(numpos_, OnlyNumber, Atom),
    atom_number(OnlyNumber, Number).

% Negative
gdli_atom_to_number(Atom, Number) :-
    atom_concat(numneg_, OnlyNumber, Atom),
    atom_concat('-', OnlyNumber, SignedNumberAtom),
    atom_number(SignedNumberAtom, Number).

% Positive
gdli_number_to_atom(Number, Atom) :-
    (1 is sign(Number); 0 is sign(Number)),
    atom_number(OnlyNumber, Number),
    atom_concat(numpos_, OnlyNumber, Atom).

% Negative
gdli_number_to_atom(Number, Atom) :-
    -1 is sign(Number),
    PNumber is -1 * Number,
    atom_number(OnlyNumber, PNumber),
    atom_concat(numneg_, OnlyNumber, Atom).

% -- Add --
% ---------

gdli_add(X, Y, Z) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    NZ is NX + NY,
    gdli_number_to_atom(NZ, Z).

gdli_add(X, Y, Z) :-
    nonvar(X),
    nonvar(Z),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Z, NZ),
    NY is NZ - NX,
    gdli_number_to_atom(NY, Y).

gdli_add(X, Y, Z) :-
    nonvar(Y),
    nonvar(Z),
    gdli_atom_to_number(Y, NY),
    gdli_atom_to_number(Z, NZ),
    NX is NZ - NY,
    gdli_number_to_atom(NX, X).


% -- Sub --
% ---------

gdli_sub(X, Y, Z) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    NZ is NX - NY,
    gdli_number_to_atom(NZ, Z).

gdli_sub(X, Y, Z) :-
    nonvar(X),
    nonvar(Z),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Z, NZ),
    NY is NX - NZ,
    gdli_number_to_atom(NY, Y).

gdli_sub(X, Y, Z) :-
    nonvar(Y),
    nonvar(Z),
    gdli_atom_to_number(Y, NY),
    gdli_atom_to_number(Z, NZ),
    NX is NY + NZ,
    gdli_number_to_atom(NX, X).


% -- Mult --
% ----------

gdli_mult(X, Y, Z) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    NZ is NX * NY,
    gdli_number_to_atom(NZ, Z).

gdli_mult(X, Y, Z) :-
    nonvar(X),
    nonvar(Z),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Z, NZ),
    NX \== 0,
    NY is NZ / NX,
    NYF is round(NY),
    gdli_number_to_atom(NYF, Y).

gdli_mult(X, Y, Z) :-
    nonvar(Y),
    nonvar(Z),
    gdli_atom_to_number(Z, NZ),
    gdli_atom_to_number(Y, NY),
    NY \== 0,
    NX is NZ / NY,
    NXF is round(NX),
    gdli_number_to_atom(NXF, X).


% -- Div --
% ---------

gdli_div(X, Y, Z) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    NY \== 0,
    NZ is NX / NY,
    NZF is round(NZ),
    gdli_number_to_atom(NZF, Z).

gdli_div(X, Y, Z) :-
    nonvar(X),
    nonvar(Z),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Z, NZ),
    NZ \== 0,
    NY is NX / NZ,
    NYF is round(NY),
    gdli_number_to_atom(NYF, Y).

gdli_div(X, Y, Z) :-
    nonvar(Y),
    nonvar(Z),
    gdli_atom_to_number(Z, NZ),
    gdli_atom_to_number(Y, NY),
    NX is NY * NZ,
    gdli_number_to_atom(NX, X).

% -- Mod --
% ---------

gdli_mod(X, Y, Z) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    NY \== 0,
    NZ is NX mod NY,
    gdli_number_to_atom(NZ, Z).

% -- Succ --
% ----------

gdli_succ(X, Y) :-
    nonvar(X),
    gdli_atom_to_number(X, NX),
    NY is NX + 1,
    gdli_number_to_atom(NY, Y).

gdli_succ(X, Y) :-
    nonvar(Y),
    gdli_atom_to_number(Y, NY),
    NX is NY - 1,
    gdli_number_to_atom(NX, X).


% -- Less --
% ----------

gdli_less(X, Y) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    NX < NY.


% -- Greater --
% -------------

gdli_greater(X, Y) :-
    gdli_less(Y, X).


% -- Between --
% -------------

gdli_between(X, Y, Z) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    between(NX, NY, NZ),
    gdli_number_to_atom(NZ, Z).

% -- Equal --
% ------------

gdli_equal(X, Y) :-
    nonvar(X),
    nonvar(Y),
    gdli_atom_to_number(X, NX),
    gdli_atom_to_number(Y, NY),
    NX = NY.


% -- Number --
% ------------

gdli_number(X) :-
    nonvar(X),
    gdli_atom_to_number(X, NX),
    integer(NX).


% ----------------------------
% --------- Mapping ----------
% ----------------------------

% -- Does --
% ----------

gdl_rule([does, X, Y]) :-
    gdli_input([X, Y]).

% -- Arithmetic --
% ----------------

gdl_rule([add, Token_X, Token_Y, Token_Z]) :-
    gdli_add(Token_X, Token_Y, Token_Z).

gdl_rule([sub, Token_X, Token_Y, Token_Z]) :-
    gdli_sub(Token_X, Token_Y, Token_Z).

gdl_rule([mult, Token_X, Token_Y, Token_Z]) :-
    gdli_mult(Token_X, Token_Y, Token_Z).

gdl_rule([div, Token_X, Token_Y, Token_Z]) :-
    gdli_div(Token_X, Token_Y, Token_Z).

gdl_rule([mod, Token_X, Token_Y, Token_Z]) :-
    gdli_mod(Token_X, Token_Y, Token_Z).

gdl_rule([succ, Token_X, Token_Y]) :-
    gdli_succ(Token_X, Token_Y).

gdl_rule([less, Token_X, Token_Y]) :-
    gdli_less(Token_X, Token_Y).

gdl_rule([greater, Token_X, Token_Y]) :-
    gdli_greater(Token_X, Token_Y).

gdl_rule([equal, Token_X, Token_Y]) :-
    gdli_equal(Token_X, Token_Y).

gdl_rule([number, Token_X]) :-
    gdli_number(Token_X).

% -- Not / Distinct --
% --------------------

gdl_not(X) :-
    \+ X.

gdl_rule([distinct | Xs]) :-
    gdli_distinct(Xs).


% -- Goal / Terminal --
% ---------------------

gdli_terminal() :-
    gdl_rule(terminal),
    !.

gdli_goal(X, Y) :-
    gdl_rule([goal, X, Y]).
