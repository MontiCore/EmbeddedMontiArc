:- use_module(library(aggregate)).
:- use_module(library(lists)).
:- style_check(-singleton).
:- style_check(-discontiguous).

% ----------------------------
% -------- Interpreter -------
% ----------------------------

gdli_with_types().

% ----------------------------
% ---------- Mapping ---------
% ----------------------------

gdlt_type(X, Y) :-
    gdl_rule([type, X, Y]).

gdlt_type(X, V) :-
    gdlt_type_combine(X, Y, Z),
    (gdlt_type(Y, V); gdlt_type(Z, V)).

gdlt_type_combine(X, Y, Z) :-
    gdl_rule([typecombine, X, Y, Z]).

gdlt_type_map(X, Y, Z) :-
    gdl_rule([typemap, X, Y, Z]),
    !.

% ----------------------------
% ------- Template Gen -------
% ----------------------------

gdl_template_state(_) :- false.
gdl_template_action(_) :- false.

gdlt_make_all_templates() :-
    gdlt_merge_all_templates(state, SMerged),
    gdlt_merge_all_templates(action, AMerged),
    gdlt_assert_all_templates(state, SMerged),
    gdlt_assert_all_templates(action, AMerged),
    !.


% -- Merge all templates

:- dynamic(gdlt_all_templates_merged/2).
gdlt_all_templates_merged(state, []) :- false.
gdlt_all_templates_merged(action, []) :- false.

gdlt_assert_all_templates(X, Y) :- 
    assertz(gdlt_all_templates_merged(X, Y)).

gdlt_merge_all_templates(Scope, Result) :-
    gdlt_all_templates(Scope, [T | Qs]),
    gdlt_merge(T, Qs, Result),
    !.
gdlt_merge_all_templates(_, []).

gdlt_all_templates(state, Templates) :-
    setof(X, gdl_template_state(X), Templates).
gdlt_all_templates(action, Templates) :-
    setof(X, gdl_template_action(X), Templates).

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
gdlt_merge_templates((range, StartY, EndY), (range, StartX, EndX), (range, StartX, EndY)) :-
    (gdli_greater(EndX, StartY); EndX = StartY; gdli_succ(EndX, StartY)),
    (gdli_less(StartX, StartY); StartX = StartY),
    !.
gdlt_merge_templates(Type, (constant, Value), Type) :-
    gdlt_type(Type, Value),
    !.
gdlt_merge_templates((constant, Value), Type, Type) :-
    gdlt_type(Type, Value),
    !.

gdlt_merge_templates(Type1, Type2, Type3) :-
    (gdlt_type_combine(Type3, Type1, Type2); gdlt_type_combine(Type3, Type2, Type1)).

gdlt_merge_templates(Type1, Type2, CombineType) :-
    (gdlt_type_combine(Temp, Type1, _); gdlt_type_combine(Temp, _, Type1)),
    gdlt_merge_templates(Temp, Type2, CombineType).

gdlt_merge_templates(Type1, Type2, CombineType) :-
    (gdlt_type_combine(Temp, Type2, _); gdlt_type_combine(Temp, _, Type2)),
    gdlt_merge_templates(Type1, Temp, CombineType).

gdlt_merge_templates(Type, CombineType, CombineType) :-
    (gdlt_type_combine(CombineType, Type, _); gdlt_type_combine(CombineType, _, Type)),
    !.
gdlt_merge_templates(CombineType, Type, CombineType) :-
    (gdlt_type_combine(CombineType, Type, _); gdlt_type_combine(CombineType, _, Type)),
    !.

gdlt_merge_templates((constant, Value1), (constant, Value2), (range, Value1, Value2)) :-
    gdli_number(Value1),
    gdli_number(Value2),
    \+ gdlt_type(_, Value1),
    \+ gdlt_type(_, Value2),
    gdli_succ(Value1, Value2),
    !.
gdlt_merge_templates((range, Start, End), (constant, Value), (range, Start, End)) :-
    gdli_number(Value),
    \+ gdlt_type(_, Value),
    (gdli_greater(Value, Start); Value = Start),
    (gdli_less(Value, End); Value = End),
    !.
gdlt_merge_templates((constant, Value), (range, Start, End), (range, Start, End)) :-
    gdli_number(Value),
    \+ gdlt_type(_, Value),
    (gdli_greater(Value, Start); Value = Start),
    (gdli_less(Value, End); Value = End),
    !.

gdlt_merge_templates((range, Start, End), (constant, Value), (range, Value, End)) :-
    gdli_number(Value),
    \+ gdlt_type(_, Value),
    gdli_succ(Value, Start),
    !.
gdlt_merge_templates((constant, Value), (range, Start, End), (range, Value, End)) :-
    gdli_number(Value),
    \+ gdlt_type(_, Value),
    gdli_succ(Value, Start),
    !.

gdlt_merge_templates((range, Start, End), (constant, Value), (range, Start, Value)) :-
    gdli_number(Value),
    \+ gdlt_type(_, Value),
    gdli_succ(End, Value),
    !.
gdlt_merge_templates((constant, Value), (range, Start, End), (range, Start, Value)) :-
    gdli_number(Value),
    \+ gdlt_type(_, Value),
    gdli_succ(End, Value),
    !.

gdlt_merge_templates([X | Xs], [Y | Ys], [Merged1 | Merged2]) :-
    gdlt_merge_templates(X, Y, Merged1),
    gdlt_merge_templates(Xs, Ys, Merged2),
    !.

% -- Dimensions
gdlt_get_dimension(X, Dimension) :-
    gdlt_all_templates_merged(X, Templates),
    gdlt_get_templates_dimension(Templates, Dimension),
    !.
gdlt_get_dimension(_, numpos_0).

% For templates
gdlt_get_templates_dimension([Type | Types], Dimension) :- 
    gdlt_get_type_dimension(Type, TDim),
    gdlt_get_templates_dimension(Types, TDims),
    gdli_add(TDim, TDims, Dimension),
    !.
gdlt_get_templates_dimension([Type], Dimension) :- 
    gdlt_get_type_dimension(Type, Dimension).

% For type
gdlt_get_type_dimension([Type | Types], Dimension) :-
    gdlt_get_type_dimension(Type, D),
    gdlt_get_type_dimension(Types, Ds),
    gdli_mult(D, Ds, Dimension),
    !.
gdlt_get_type_dimension([Type], Dimension) :-
    gdlt_get_type_dimension(Type, Dimension),
    !.
gdlt_get_type_dimension(Type, Dimension) :-
    setof(X, gdlt_type(Type, X), All),
    length(All, NDimension),
    gdli_number_to_atom(NDimension, Dimension),
    !.
gdlt_get_type_dimension((constant, _), numpos_1) :- !.
gdlt_get_type_dimension((range, Start, End), Dimension) :-
    gdli_sub(End, Start, Sub),
    gdli_add(Sub, numpos_1, Dimension).


% -- Create Indicator matrix
gdlt_role_indicator_matrix(Role, Matrix) :-
    gdli_full_state_role(Role, Models),
    maplist(gdlt_indicator_map, Models, Matrix),
    !.

gdlt_indicator_map(State, Index) :-
    gdlt_index_map(state, State, Index).


% -- Index mapping
gdlt_all_index_value_pairs(Scope, AllMaps) :-
    setof([Index, Value], gdlt_index_map(Scope, Value, Index), AllMaps).

gdlt_index_map(Scope, Value, Index) :-
    nonvar(Value),
    gdlt_value_index(Scope, Value, Index),
    !.
gdlt_index_map(Scope, Value, Index) :-
    nonvar(Index),
    gdlt_index_value(Scope, Index, Value),
    !.
gdlt_index_map(Scope, Value, Index) :-
    var(Value),
    var(Index),
    gdlt_all_templates_merged(Scope, Templates),
    gdlt_get_templates_dimension(Templates, Dimension),
    gdli_sub(Dimension, numpos_1, DSub),
    gdli_between(numpos_0, DSub, Index),
    gdlt_index_value(Scope, Index, Value).

% Index is variable
% Map Value to Index
gdlt_value_index(Scope, Value, Index) :-
    gdlt_all_templates_merged(Scope, Templates),
    nth0(_, Templates, Template),
    gdlt_value_template_index(Value, Template, ValueIndex),
    gdlt_template_start_index(Scope, Template, TypeIndex),
    gdli_add(TypeIndex, ValueIndex, Index).


gdlt_template_start_index(Scope, Template, Index) :-
    gdlt_all_templates_merged(Scope, Templates),
    gdlt_template_start_index_r(Template, Templates, Index).

gdlt_template_start_index_r(T, [T], numpos_0).
gdlt_template_start_index_r(T, [T | Ts], Index) :-
    gdlt_get_templates_dimension(Ts, Index).
gdlt_template_start_index_r(T, [Tx | Ts], Index) :-
    gdlt_template_start_index_r(T, Ts, Index).


gdlt_value_template_index(Value, Type, Index) :-
    var(Index),
    gdlt_type_map(Type, TypeInstance, Value),
    setof(X, gdlt_type(Type, X), InstanceValues),
    nth0(NIndex, InstanceValues, TypeInstance),
    gdli_number_to_atom(NIndex, Index),
    !.

gdlt_value_template_index(Value, Type, Index) :-
    var(Index),
    setof(X, gdlt_type(Type, X), Values),
    nth0(NIndex, Values, Value),
    gdli_number_to_atom(NIndex, Index),
    !.
gdlt_value_template_index(Value, Type, Index) :-
    nonvar(Index),
    gdli_atom_to_number(Index, NIndex),
    setof(X, gdlt_type(Type, X), Values),
    nth0(NIndex, Values, Value),
    !.
gdlt_value_template_index(Value, (constant, Value), numpos_0) :- !.
gdlt_value_template_index(Value, (range, Start, End), Index) :-
    (gdli_greater(Value, Start); Value = Start),
    (gdli_less(Value, End); Value = End),
    gdli_sub(Value, Start, Index),
    !.
gdlt_value_template_index([Value], [Template], VIndex) :-
    gdlt_value_template_index(Value, Template, VIndex),
    !.
gdlt_value_template_index([V | Vs], [T | Ts], VIndex) :-
    gdlt_value_template_index(V, T, SelfIndex),
    gdlt_value_template_index(Vs, Ts, RightIndex),
    gdlt_get_type_dimension(Ts, RightSize),
    gdli_mult(SelfIndex, RightSize, Prod),
    gdli_add(Prod, RightIndex, VIndex),
    !.


% Value is variable
% Map value to Index
gdlt_index_value(Scope, Index, Value) :-
    gdlt_all_templates_merged(Scope, Templates),
    nth0(_, Templates, Template),
    gdlt_template_start_index(Scope, Template, TypeIndex),
    gdli_sub(Index, TypeIndex, ValueIndex),
    gdlt_template_index_value(Template, ValueIndex, Value).


gdlt_template_index_value(Type, Index, Value) :-
    var(Index),
    setof(X, gdlt_type(Type, X), Values),
    nth0(NIndex, Values, Value),
    gdli_number_to_atom(NIndex, Index),
    !.
gdlt_template_index_value(Type, Index, Value) :-
    nonvar(Index),
    gdli_atom_to_number(Index, NIndex),
    setof(X, gdlt_type(Type, X), Values),
    nth0(NIndex, Values, Value),
    !.
gdlt_template_index_value((constant, Value), numpos_0, Value) :- !.
gdlt_template_index_value((range, Start, End), Index, Value) :-
    gdli_add(Start, Index, Value),
    (gdli_greater(Value, Start); Value = Start),
    (gdli_less(Value, End); Value = End),
    !.
gdlt_template_index_value([Template], Index, [Value]) :-
    gdlt_template_index_value(Template, Index, Value),
    !.
gdlt_template_index_value([T | Ts], Index, [V | Vs]) :-
    gdlt_get_type_dimension(Ts, RightSize),
    gdli_atom_to_number(Index, NIndex),
    gdli_atom_to_number(RightSize, NRightSize),
    NSelfIndex is floor(NIndex/NRightSize),
    gdli_number_to_atom(NSelfIndex, SelfIndex),
    gdlt_template_index_value(T, SelfIndex, V),
    gdli_mult(SelfIndex, RightSize, Prod),
    gdli_sub(Index, Prod, RightIndex),
    gdlt_template_index_value(Ts, RightIndex, Vs),
    !.
