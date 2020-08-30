from sympy import symbols, Function, dsolve, sympify, Derivative, linsolve, solve, solveset, nonlinsolve
import argparse


def reduceSystem(equs):
    differentialEquations = []
    algebraicEquations = []
    differentialFunctions = []
    algebraicFunctions = []

    for eq in equs:
        ds = eq.atoms(Derivative)
        if len(ds) == 0:
            algebraicEquations.append(eq)
        else:
            differentialEquations.append(eq)
            for d in ds:
                fs = d.atoms(Function)
                differentialFunctions.extend(fs)

    if len(algebraicEquations) == 0:
        return differentialEquations

    for eq in differentialEquations:
        fs = eq.atoms(Function)
        for f in fs:
            if not f in differentialFunctions:
                algebraicFunctions.append(f)

    if len(algebraicFunctions) == 0:
        return differentialEquations

    sol = solve(algebraicEquations, algebraicFunctions)
    # check if solution is present

    newEqus = []
    for eq in differentialEquations:
        for f, s in sol.items():
            eq = eq.subs(f, s)
        newEqus.append(eq)

    return newEqus


def main(equations, symbolNames, functionNames, startValues):
    symbols_ = {}
    for symbolName in symbolNames:
        symbols_[symbolName] = symbols(symbolName)

    functions_ = {}
    for functionName in functionNames:
        functions_[functionName] = Function(functionName)

    ics = {}
    for functionName in startValues.keys():
        function = functions_[functionName]
        ics[function(0)] = startValues[functionName]

    equs = sympify(equations, locals(), evaluate=False)
    equs = reduceSystem(equs)
    solutions = dsolve(equs, ics=ics)

    for solution in solutions:
        print(str(solution.args[0]).replace('(' + time + ')', ''), "=", solution.args[1])


parser = argparse.ArgumentParser(description='Solve DAE symbolic if possible')
parser.add_argument('system', metavar='eq', type=str, nargs='+')
parser.add_argument('--functions', metavar='f', type=str, nargs='+')
parser.add_argument('--y0', metavar='y0', type=float, nargs='+')
args = parser.parse_args()

if (len(args.y0) > len(args.functions)):
    raise Exception('Arguments for y0 cannot be larger than arguments for functions')

ics = {}
for i in range(len(args.y0)):
    ics[args.functions[i]] = args.y0[i]

time = 't'
system = []
for eq in args.system:
    newEq = eq
    for f in args.functions:
        newEq = newEq.replace(f, f + '(' + time + ')')
    system.append(newEq)

main(system, [time], args.functions, ics)