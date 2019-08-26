/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem;

import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;

public class SymbolToProblemConverter {
    private static SymbolToProblemConverter ourInstance = new SymbolToProblemConverter();

    public static SymbolToProblemConverter getInstance() {
        return ourInstance;
    }

    private SymbolToProblemConverter() {
    }

    public DNLPProblem getDNLPFromSymbol(MathOptimizationStatementSymbol symbol) {
        DNLPProblem dnlp = new DNLPProblem();
        ProblemAssignmentHandler.getInstance().getProblemFromSymbol(dnlp, symbol);
        return dnlp;
    }

    public NLPProblem getNLPFromSymbol(MathOptimizationStatementSymbol symbol) {
        NLPProblem nlp = new NLPProblem();
        ProblemAssignmentHandler.getInstance().getProblemFromSymbol(nlp, symbol);
        return nlp;
    }

    public LPProblem getLPFromSymbol(MathOptimizationStatementSymbol symbol) {
        LPProblem lp = new LPProblem();
        ProblemAssignmentHandler.getInstance().getProblemFromSymbol(lp, symbol);
        return lp;
    }

    public MIPProblem getMIPFromSymbol(MathOptimizationStatementSymbol symbol) {
        MIPProblem mip = new MIPProblem();
        ProblemAssignmentHandler.getInstance().getProblemFromSymbol(mip, symbol);
        return mip;
    }

    public MIQPProblem getMIQPFromSymbol(MathOptimizationStatementSymbol symbol) {
        MIQPProblem miqp = new MIQPProblem();
        ProblemAssignmentHandler.getInstance().getProblemFromSymbol(miqp, symbol);
        return miqp;
    }

    public QPProblem getQPFromSymbol(MathOptimizationStatementSymbol symbol) {
        QPProblem qp = new QPProblem();
        ProblemAssignmentHandler.getInstance().getProblemFromSymbol(qp, symbol);
        return qp;
    }
}
