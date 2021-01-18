import argparse
from sympy import symbols, sympify, linsolve


def main(equations, symbolNames):
    symbols_ = {}
    for symbolName in symbolNames:
        symbols_[symbolName] = symbols(symbolName)

    equs = sympify(equations, locals(), evaluate=False)
    solution = linsolve(equs, symbols_)

    i = 0
    for sol in solution.args[0]:
        print(symbolNames[i], "=", sol)
        i = i + 1


parser = argparse.ArgumentParser(description='Solve DAE symbolic if possible')
parser.add_argument('system', metavar='eq', type=str, nargs='+')
parser.add_argument('--symbols', metavar='y', type=str, nargs='+')
args = parser.parse_args()

main(args.system, args.symbols)