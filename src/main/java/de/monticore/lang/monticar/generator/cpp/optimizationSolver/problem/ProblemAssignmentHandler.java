/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.optimizationSolver.problem;

import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.monticar.generator.Generator;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.converter.*;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Vector;

import static de.monticore.lang.monticar.generator.cpp.optimizationSolver.problem.NLPProblem.LOWER_BOUND_INF;
import static de.monticore.lang.monticar.generator.cpp.optimizationSolver.problem.NLPProblem.UPPER_BOUND_INF;

public class ProblemAssignmentHandler {
    private static ProblemAssignmentHandler ourInstance = new ProblemAssignmentHandler();

    static ProblemAssignmentHandler getInstance() {
        return ourInstance;
    }

    private ProblemAssignmentHandler() {
    }

    public void getProblemFromSymbol(Problem problem, MathOptimizationStatementSymbol symbol) {
        problem.setId(symbol.getExpressionID());
        problem.setOptimizationProblemType(symbol.getOptimizationType());
        // assign all properties
        setOptimizationVariableFromSymbol(problem, symbol);
        setObjectiveFunctionFromSymbol(problem, symbol);
        setConstraintsFromSymbol(problem, symbol);
    }

    private void setOptimizationVariableFromSymbol(Problem p, MathOptimizationStatementSymbol symbol) {

        Vector<Integer> dimensions = new Vector<>();
        p.setN(getOptimizationVarDimension(symbol, dimensions));
        p.setOptimizationVariableName(getOptimizationVarName(symbol));
        p.setOptimizationVariableType(getOptimizationVarType(symbol));
        p.setOptimizationVariableDimensions(dimensions);

        getOptimizationSymbolHandler().setCurrentOptimizationVariableName(getOptimizationVarName(symbol));
    }

    private void setObjectiveFunctionFromSymbol(Problem p, MathOptimizationStatementSymbol symbol) {
        p.setObjectiveValueVariable(getObjectiveValueVarName(symbol));
        p.setObjectiveFunction(getObjectiveFunctionAsCode(symbol));
    }

    private String[] getBoundsFromConstraint(Problem p, MathOptimizationConditionSymbol constraint) {
        String lowerBound = LOWER_BOUND_INF;
        String upperBound = UPPER_BOUND_INF;
        if (constraint.getLowerBound().isPresent())
            lowerBound = ExecuteMethodGenerator.generateExecuteCode(constraint.getLowerBound().get(), new ArrayList<String>());
        if (constraint.getUpperBound().isPresent())
            upperBound = ExecuteMethodGenerator.generateExecuteCode(constraint.getUpperBound().get(), new ArrayList<String>());
        return new String[]{lowerBound, upperBound};
    }

    private boolean isConstraintOnOptVar(Problem p, MathExpressionSymbol expr) {
        String name = "";
        if (expr instanceof MathNameExpressionSymbol) {
            name = ((MathNameExpressionSymbol) expr).getNameToResolveValue();
        } else if (expr instanceof MathMatrixNameExpressionSymbol) {
            name = ((MathMatrixNameExpressionSymbol) expr).getNameToAccess();
        } else {
            name = "";
        }
        return name.contentEquals(p.getOptimizationVariableName());
    }

    private void mergeBoundsInX(Problem p, Vector<String> xL, Vector<String> xU, MathExpressionSymbol expr, String currXL, String currXU, Vector<String> xMatrixElementConstraints) {
        if (expr instanceof MathMatrixNameExpressionSymbol) {
            MathMatrixNameExpressionSymbol matNameExpr = (MathMatrixNameExpressionSymbol) expr;
            String indexExpr = ExecuteMethodGeneratorMatrixExpressionHandler.generateExecuteCode(matNameExpr.getMathMatrixAccessOperatorSymbol(), new ArrayList<>());
            if (indexExpr.contains(".col") && indexExpr.contains(".row"))
                indexExpr = "all";
            else if (indexExpr.startsWith(".col"))
                indexExpr = "\"col\", " + indexExpr.substring(indexExpr.indexOf("(") + 1, indexExpr.indexOf(")"));
            else if (indexExpr.startsWith(".row"))
                indexExpr = "\"row\", " + indexExpr.substring(indexExpr.indexOf("(") + 1, indexExpr.indexOf(")"));
            else {
                Vector<Integer> dims = p.getOptimizationVariableDimensions();
                if (indexExpr.contains(",")) {
                    indexExpr = indexExpr.replaceAll(",", "+ (");
                    StringBuilder indexSB = new StringBuilder();
                    indexSB.append(indexExpr);
                    indexSB.insert(indexSB.length() - 1 ," - 1 ) * " + dims.get(0) + " ");
                    indexExpr = indexSB.toString();
                }
                if (MathConverter.curBackend.usesZeroBasedIndexing()) {
                    if (indexExpr.contains("+"))
                        indexExpr = indexExpr.replaceAll("\\+", "- 1 +");
                    else if (indexExpr.contains(")"))
                        indexExpr = indexExpr.substring(0, indexExpr.lastIndexOf(")")) + "- 1)";
                }
            }
            String funcSigniture = String.format("%s, %s, %s", currXL, indexExpr, currXU);
            xMatrixElementConstraints.add(funcSigniture);
        } else if (expr instanceof MathNameExpressionSymbol) {
            for (int i = 0; i < p.getN(); i++) {
                xL.set(i, String.format("std::fmax(%s, %s)", xL.get(i), currXL));
                xU.set(i, String.format("std::fmin(%s, %s)", xU.get(i), currXU));
            }
        } else {
            Log.error("Function mergeBoundsInX: Constraint expression must be an optimization variable here.");
        }
    }

    private void setBoundsOnXFromTypeDeclaration(MathOptimizationStatementSymbol symbol, Vector<String> xL, Vector<String> xU, int n) {
        String lowerBoundX = LOWER_BOUND_INF;
        String upperBoundX = UPPER_BOUND_INF;

        MathValueType type = getVariableWithTypeInformations(symbol.getOptimizationVariable()).getType();
        if (type.getType().getRangeOpt().isPresent()) {
            lowerBoundX = Double.toString(type.getType().getRange().getStartValue().doubleValue());
            upperBoundX = Double.toString(type.getType().getRange().getEndValue().doubleValue());
        }
        for (int i = 0; i < n; i++) {
            xL.add(lowerBoundX);
            xU.add(upperBoundX);
        }
    }

    private void setConstraintsFromSymbol(Problem p, MathOptimizationStatementSymbol symbol) {
        Vector<String> g = new Vector<>();
        Vector<String> gL = new Vector<>();
        Vector<String> gU = new Vector<>();
        Vector<String> xL = new Vector<>();
        Vector<String> xU = new Vector<>();
        Vector<String> xMatrixElementConstraints = new Vector<>();
        // add constraints
        setBoundsOnXFromTypeDeclaration(symbol, xL, xU, p.getN());
        addConstraintsOnObjectiveVariable(symbol, g, gL, gU);
        addSubjectToConstraints(p, symbol, xL, xU, g, gL, gU, xMatrixElementConstraints);
        // set nlp
        p.setM(g.size());
        p.setConstraintFunctions(g);
        p.setgL(gL);
        p.setgU(gU);
        p.setxL(xL);
        p.setxU(xU);
        p.setXMatrixElementConstraints(xMatrixElementConstraints);
    }

    private void addSubjectToConstraints(Problem p, MathOptimizationStatementSymbol symbol, Vector<String> xL, Vector<String> xU, Vector<String> g, Vector<String> gL, Vector<String> gU, Vector<String> xMatrixElementConstraints) {
        for (MathExpressionSymbol constraint : symbol.getSubjectToExpressions()) {
            // find function
            if (constraint instanceof MathOptimizationConditionSymbol) {
                MathOptimizationConditionSymbol singleConstraint = (MathOptimizationConditionSymbol) constraint;
                addSingleConstraint(p, g, gL, gU, xL, xU, singleConstraint, xMatrixElementConstraints);
            } else if (constraint instanceof MathForLoopExpressionSymbol) {
                MathForLoopExpressionSymbol loopConstraint = (MathForLoopExpressionSymbol) constraint;
                addLoopConstraints(p, g, gL, gU, xL, xU, loopConstraint, xMatrixElementConstraints);
            }
        }
    }

    private void addConstraintsOnObjectiveVariable(MathOptimizationStatementSymbol symbol, Vector<String> g, Vector<String> gL, Vector<String> gU) {
        if (symbol.hasReturnValue()) {
            MathValueSymbol objval = getVariableWithTypeInformations(symbol.getObjectiveValue());
            if (objval.getType() != null) {
                Optional<ASTRange> rangeOpt = objval.getType().getType().getRangeOpt();
                if (rangeOpt.isPresent() && rangeOpt.get().getMin().getNumber().isPresent() && rangeOpt.get().getMax().getNumber().isPresent()) {
                    Double min = rangeOpt.get().getMin().getNumber().get();
                    Double max = rangeOpt.get().getMax().getNumber().get();
                    gL.add(min.toString());
                    gU.add(max.toString());
                    g.add(getObjectiveFunctionAsCode(symbol));
                }
            }
        }
    }

    private void addLoopConstraints(Problem p, Vector<String> g, Vector<String> gL, Vector<String> gU, Vector<String> xL, Vector<String> xU, MathForLoopExpressionSymbol loopConstraint, Vector<String> xMatrixElementConstraints) {
        Optional<MathExpressionSymbol> startExpr = ForLoopHeadConverter.getForLoopStart(loopConstraint.getForLoopHead());
        Optional<MathExpressionSymbol> endExpr = ForLoopHeadConverter.getForLoopEnd(loopConstraint.getForLoopHead());
        Optional<MathExpressionSymbol> stepExpr = ForLoopHeadConverter.getForLoopStep(loopConstraint.getForLoopHead());
        if (startExpr.isPresent() && endExpr.isPresent()) {
            Optional<Double> startValue = ComponentConverter.currentBluePrint.getMathInformationRegister().tryGetDoubleValue(startExpr.get());
            Optional<Double> endValue = ComponentConverter.currentBluePrint.getMathInformationRegister().tryGetDoubleValue(endExpr.get());
            if (startValue.isPresent() && endValue.isPresent()) {
                double loopStart = startValue.get();
                double loopEnd = endValue.get();
                double loopStep = 1;
                if (stepExpr.isPresent()) {
                    Optional<Double> stepValue = ComponentConverter.currentBluePrint.getMathInformationRegister().tryGetDoubleValue(stepExpr.get());
                    if (stepValue.isPresent())
                        loopStep = stepValue.get();
                    else
                        errorMessageForLoopConstraint(loopConstraint);
                }
                for (double i = loopStart; i <= loopEnd; i += loopStep) {
                    for (MathExpressionSymbol constraint : loopConstraint.getForLoopBody()) {
                        if (constraint instanceof MathOptimizationConditionSymbol) {
                            int size = g.size();
                            addSingleConstraint(p, g, gL, gU, xL, xU, (MathOptimizationConditionSymbol) constraint, xMatrixElementConstraints);
                            if (!isConstraintOnOptVar(p, constraint) && (g.size() == size + 1)) {
                                g.set(g.size() - 1, replaceLoopVariable(g.lastElement(), loopConstraint.getForLoopHead().getNameLoopVariable(), Double.toString(i)));
                            } else {
                                xMatrixElementConstraints.set(xMatrixElementConstraints.size() - 1, replaceLoopVariable(xMatrixElementConstraints.lastElement(), loopConstraint.getForLoopHead().getNameLoopVariable(), Double.toString(i)));
                            }
                        } else if (constraint instanceof MathForLoopExpressionSymbol) {
                            addLoopConstraints(p, g, gL, gU, xL, xU, (MathForLoopExpressionSymbol) constraint, xMatrixElementConstraints);
                        }
                    }
                }
            } else {
                errorMessageForLoopConstraint(loopConstraint);
            }
        } else {
            errorMessageForLoopConstraint(loopConstraint);
        }
    }

    private String replaceLoopVariable(String expr, String loopVar, String replacement) {
        expr = expr.replaceAll("[()+\\-*/^']", " $0 ");
        return expr.replaceAll(" " + loopVar + " ", " " + replacement + " ");
    }

    private void errorMessageForLoopConstraint(MathForLoopExpressionSymbol loopExpressionSymbol) {
        Log.error(String.format("Cannot resolve value of \"%s\"", loopExpressionSymbol.getForLoopHead().getTextualRepresentation()), loopExpressionSymbol.getForLoopHead().getSourcePosition());
    }

    private void addSingleConstraint(Problem p, Vector<String> g, Vector<String> gL, Vector<String> gU, Vector<String> xL, Vector<String> xU, MathOptimizationConditionSymbol singleConstraint, Vector<String> xMatrixElementConstraints) {
        MathExpressionSymbol expr = singleConstraint.getBoundedExpression();
        String[] bounds = getBoundsFromConstraint(p, singleConstraint);
        if (isConstraintOnOptVar(p, expr)) {
            mergeBoundsInX(p, xL, xU, expr, bounds[0], bounds[1], xMatrixElementConstraints);
        } else {
            MathFunctionFixer.fixMathFunctions(expr, ComponentConverter.currentBluePrint);
            String gAsString = ExecuteMethodGenerator.generateExecuteCode(expr, new ArrayList<>());
            g.add(gAsString);
            gL.add(bounds[0]);
            gU.add(bounds[1]);
        }
    }

    private MathNumberExpressionSymbol getNumber(MathExpressionSymbol symbol) {
        MathNumberExpressionSymbol numberExpr = null;
        if (symbol.isValueExpression()) {
            if (((MathValueExpressionSymbol) symbol).isNumberExpression()) {
                numberExpr = ((MathNumberExpressionSymbol) symbol);
            }
        }
        return numberExpr;
    }

    private int getOptimizationVarDimension(MathOptimizationStatementSymbol symbol, Vector<Integer> dimensions) {
        int n = 1;
        dimensions.clear();
        MathValueSymbol optVarDeclaration = getVariableWithTypeInformations(symbol.getOptimizationVariable());
        List<MathExpressionSymbol> dims = optVarDeclaration.getType().getDimensions();
        for (MathExpressionSymbol d : dims) {
            if (getNumber(d) != null) {
                int currDim = getNumber(d).getValue().getRealNumber().intValue();
                n *= currDim;
                dimensions.add(currDim);
            }
        }
        return n;
    }

    private String getOptimizationVarName(MathOptimizationStatementSymbol symbol) {
        return symbol.getOptimizationVariable().getName();
    }

    public MathValueSymbol getVariableWithTypeInformations(MathValueSymbol symbol) {
        return ComponentConverter.currentBluePrint.getMathInformationRegister().getFullTypeInformation(symbol);
    }

    private String getOptimizationVarType(MathOptimizationStatementSymbol symbol) {
        return TypeConverter.getVariableTypeNameForMathLanguageTypeName(getVariableWithTypeInformations(symbol.getOptimizationVariable()).getType());
    }

    private String getObjectiveValueVarName(MathOptimizationStatementSymbol symbol) {
        String objValueVar = "";
        if (symbol.hasReturnValue()) {
            objValueVar = symbol.getObjectiveValue().getName();
        }
        return objValueVar;
    }

    private String getObjectiveFunctionAsCode(MathOptimizationStatementSymbol symbol) {
        MathExpressionSymbol substitutedObjFunc = ComponentConverter.currentBluePrint.getMathInformationRegister().resolveMathExpressionToAtomarExpression(symbol.getObjectiveExpression().getAssignedMathExpressionSymbol(), symbol.getOptimizationVariable().getName());
        MathFunctionFixer.fixMathFunctions(substitutedObjFunc, ComponentConverter.currentBluePrint);
        return ExecuteMethodGenerator.generateExecuteCode(substitutedObjFunc, new ArrayList<>());
    }

    private OptimizationSymbolHandler getOptimizationSymbolHandler() {
        Generator gen = ComponentConverter.currentBluePrint.getGenerator();
        if (gen instanceof GeneratorEMAMOpt2CPP)
            return ((GeneratorEMAMOpt2CPP) gen).getExecuteMethodGeneratorOpt();
        else
            return null;
    }
}
