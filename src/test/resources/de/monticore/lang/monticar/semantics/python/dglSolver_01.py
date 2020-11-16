from sympy import symbols, Function, Eq, dsolve

y = symbols('y', cls=Function)
t = symbols('t')

m = 2
k = 3

diffeq = Eq(y(t).diff(t).diff(t)*m, -k*y(t))

sol = dsolve(diffeq,ics={y(0):2,y(t).diff(t).subs(t,0):1})

print(sol)
print(sol.free_symbols)