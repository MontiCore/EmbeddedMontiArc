:- use_module(library(aggregate)).
:- use_module(library(lists)).
:- style_check(-singleton).
:- style_check(-discontiguous).

gdl_with_types().

gdlt_type(X, Y) :-
    gdl_rule([type, X, Y]).

% ----------------------------
% ------- Template Gen -------
% ----------------------------

% INIT ONCE
gdlt_make_templates() :-
    gdlt_merge_all_templates(state, StateMerged),
    gdlt_assert_all_templates(state, StateMerged),
    gdlt_merge_all_templates(action, ActionMerged),
    gdlt_assert_all_templates(action, ActionMerged).


% "PUBLIC"
gdlt_get_state_index(Value, Index) :-
    gdlt_template_state_start_index(Template, TIndex),
    gdlt_value_template_index(Value, Template, VIndex),
    gdli_add(TIndex, VIndex, Index),
    !.
gdlt_get_action_index(Value, Index) :-
    gdlt_template_action_start_index(Template, TIndex),
    gdlt_value_template_index(Value, Template, VIndex),
    gdli_add(TIndex, VIndex, Index),
    !.


gdlt_get_state_dimension(Dimension) :-
    gdlt_all_templates_merged(state, [T | _]),
    gdlt_template_state_start_index(T, I),
    gdlt_template_size(T, S),
    gdli_add(I, S, Dimension),
    !.
gdlt_get_state_dimension(Dimension) :-
    gdlt_all_templates_merged(state, [T]),
    gdlt_template_size(T, Dimension),
    !.
gdlt_get_state_dimension(numpos_0).


gdlt_get_action_dimension(Dimension) :-
    gdlt_all_templates_merged(action, [T | _]),
    gdlt_template_action_start_index(T, I),
    gdlt_template_size(T, S),
    gdli_add(I, S, Dimension),
    !.
gdlt_get_action_dimension(Dimension) :-
    gdlt_all_templates_merged(action, [T]),
    gdlt_template_size(T, Dimension),
    !.
gdlt_get_action_dimension(numpos_0).


% "PRIVATE"
gdlt_all_state_templates(Templates) :-
    setof(X, gdl_template_state(X), Templates).
gdlt_all_action_templates(Templates) :-
    setof(X, gdl_template_action(X), Templates).


:- dynamic(gdlt_all_templates_merged/2).
gdlt_all_templates_merged(state, []) :- false.
gdlt_all_templates_merged(action, []) :- false.

gdlt_assert_all_templates(X, Y) :- 
    assertz(gdlt_all_templates_merged(X, Y)).

gdlt_merge_all_templates(state, Result) :-
    gdlt_all_state_templates([T | Qs]),
    gdlt_merge(T, Qs, Result).
gdlt_merge_all_templates(action, Result) :-
    gdlt_all_action_templates([T | Qs]),
    gdlt_merge(T, Qs, Result).

gdlt_merge(T, [], [T]) :- !.
gdlt_merge(T, [Q], [M]) :-
    gdlt_merge_templates(T, Q, M), !.
gdlt_merge(T, [Q], [T, Q]) :-
    \+ gdlt_merge_templates(T, Q, _).
gdlt_merge(T, [Q | Qs], Ms) :-
    gdlt_merge_templates(T, Q, M),
    gdlt_merge(M, Qs, Ms), !.
gdlt_merge(T, [Q | Qs], [T | Ms]) :-
    \+ gdlt_merge_templates(T, Q, _),
    gdlt_merge(Q, Qs, Ms), !.

gdlt_merge_templates(Temp, Temp, Temp) :- !.
gdlt_merge_templates((range, StartX, EndX), (range, StartY, EndY), (range, StartX, EndY)) :-
    (gdli_greater(EndX, StartY); EndX = StartY),
    (gdli_less(StartX, StartY); StartX = StartY),
    !.
gdlt_merge_templates((range, StartY, EndY), (range, StartX, EndX), (range, StartX, EndY)) :-
    (gdli_greater(EndX, StartY); EndX = StartY),
    (gdli_less(StartX, StartY); StartX = StartY),
    !.
gdlt_merge_templates([X | Xs], [Y | Ys], [Merged1 | Merged2]) :-
    gdlt_merge_templates(X, Y, Merged1),
    gdlt_merge_templates(Xs, Ys, Merged2),
    !.

gdlt_template_state_start_index(Template, TIndex) :-
    gdlt_all_templates_merged(state, TList),
    nth0(_, TList, Template),
    gdlt_template_start_index(Template, TList, TIndex).

gdlt_template_action_start_index(Template, TIndex) :-
    gdlt_all_templates_merged(action, TList),
    nth0(_, TList, Template),
    gdlt_template_start_index(Template, TList, TIndex).

gdlt_template_start_index(T, [T], numpos_0) :- !.
gdlt_template_start_index(T, [T | _], numpos_0) :- !.
gdlt_template_start_index(T, [Tx | Ts], Size) :-
    gdlt_template_size(Tx, XSize),
    gdlt_template_start_index(T, Ts, SSize),
    gdli_add(XSize, SSize, Size).


gdlt_value_template_index(Value, Template, VIndex) :-
    gdlt_type_index(Template, Value, VIndex),
    !.

gdlt_value_template_index([Value], [Template], VIndex) :-
    gdlt_value_template_index(Value, Template, VIndex),
    !.
gdlt_value_template_index([V | Vs], [T | Ts], VIndex) :-
    gdlt_value_template_index(V, T, SelfIndex),
    gdlt_value_template_index(Vs, Ts, RightIndex),
    gdlt_template_size(Ts, RightSize),
    gdli_mult(SelfIndex, RightSize, X1),
    gdli_add(X1, RightIndex, VIndex),
    !.


gdlt_type_index((range, Start, End), Value, Index) :-
    gdli_number(Value),
    (gdli_greater(Value, Start); Value = Start),
    (gdli_less(Value, End); Value = End),
    gdli_sub(Value, Start, Index),
    !.
gdlt_type_index((constant, undefined, Value), Value, numpos_0) :- !.
gdlt_type_index((constant, Type, Value), Value, numpos_0) :- 
    gdlt_type(Type, Value),
    !.

gdlt_type_index(Type, Value, Index) :-
    setof(X, gdlt_type(Type, X), All),
    nth0(IndexA, All, Value),
    gdli_number_to_atom(IndexA, Index),
    !.

gdlt_template_size((range, Start, End), Size) :-
    gdli_sub(End, Start, X1),
    gdli_add(X1, numpos_1, Size),
    !.
gdlt_template_size((constant, _, _), numpos_1) :-
    !.
gdlt_template_size(Template, Size) :-
    setof(X, gdlt_type(Template, X), All),
    length(All, SizeA),
    gdli_number_to_atom(SizeA, Size),
    !.

gdlt_template_size([Template], Size) :-
    gdlt_template_size(Template, Size),
    !.

gdlt_template_size([T | Ts], Size) :-
    gdlt_template_size(T, S1),
    gdlt_template_size(Ts, S2),
    gdli_mult(S1, S2, Size),
    !.
