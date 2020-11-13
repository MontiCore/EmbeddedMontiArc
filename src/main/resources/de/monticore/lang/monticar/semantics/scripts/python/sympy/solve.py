import argparse
from sympy import symbols, sympify, solve


def main(equations, symbolNames):
    symbols_ = {}
    for symbolName in symbolNames:
        symbols_[symbolName] = symbols(symbolName)

    equs = sympify(equations, locals(), evaluate=False)
    solutions = solve(equs, symbols_)

    i = 0
    for sol in solutions.keys():
        print(sol, "=", solutions[sol])


parser = argparse.ArgumentParser(description='Solve DAE symbolic if possible')
parser.add_argument('system', metavar='eq', type=str, nargs='+')
parser.add_argument('--symbols', metavar='y', type=str, nargs='+')
args = parser.parse_args()

main(args.system, args.symbols)