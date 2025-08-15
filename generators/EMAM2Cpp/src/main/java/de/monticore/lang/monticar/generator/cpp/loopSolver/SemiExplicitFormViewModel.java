/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolParentAwareVisitor;
import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathParenthesisExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.analyze.SpecificationConverter;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class SemiExplicitFormViewModel extends ViewModelBase {

    private String name;

    List<String> includes = new ArrayList<>();
    List<String> variableWithType = new ArrayList<>();
    List<String> inits = new ArrayList<>();

    private List<String> inports = new ArrayList<>();
    
    private List<String> massMatrixDiag = new ArrayList<>();
    private List<String> functions = new ArrayList<>();

    private boolean isAlgebraic = true;

    private double atol;
    private double rtol;
    private double jtol;
    private double dt_init;
    private double dt_max;
    private double loggingLevel;
    
    

    private void setOptions() {
        this.atol = NumericSolverOptions.ATOL;
        this.rtol = NumericSolverOptions.RTOL;
        this.jtol = NumericSolverOptions.JTOL;
        this.loggingLevel = NumericSolverOptions.LEVEL_LOGGING;
        this.dt_init = NumericSolverOptions.DT_SOLVER;
        this.dt_max = NumericSolverOptions.DT_SOLVER;
    }

    public SemiExplicitFormViewModel(SemiExplicitForm semiExplicitForm) {
        setOptions();
        this.name = name;
//        for (EMAPortInstanceSymbol inport : inports) {
//            this.inports.add(CPPEquationSystemHelper.getNameOfPort(inport));
//        }
//
//        for (Variable variable : bluePrint.getVariables()) {
//            if (variable.isPublic()) {
//                includes.add(variable.getVariableType().getIncludeName());
//                variableWithType.add(String.format("%s %s", variable.getVariableType().getTypeNameTargetLanguage(),
//                        variable.getName()));
//            }
//        }
//
//        Optional<Method> init = bluePrint.getMethod("init");
//        if (init.isPresent()) {
//            for (Instruction instruction : init.get().getInstructions()) {
//                inits.add(instruction.getTargetLanguageInstruction());
//            }
//        }
    }

    public SemiExplicitFormViewModel(EMAEquationSystem eqs) {
        setOptions();
        this.name = eqs.getName();

//        Map<String, String> inportMapping = new HashMap<>();
//        for (EMAPortInstanceSymbol inport : eqs.getIncomingPorts()) {
//            Optional<EMAPortInstanceSymbol> source = eqs.getAtomicSourceOf(inport);
//            String nameOfPort = CPPEquationSystemHelper.getNameOfPort(source.get());
//            inports.add(nameOfPort);
//            inportMapping.put(NameHelper.calculateFullQualifiedNameOf(source.get()), nameOfPort);
//        }
//
//        Map<String, Integer> variableIndex = new HashMap<>();
//        Integer currentIndex = 0;
//
//        for (EMAComponentInstanceSymbol component : eqs.getComponentInstanceSymbols()) {
//            Collection<EMAMInitialValueSymbol> initialValues = SpecificationConverter.getInitialValues(eqs, component);
//            Collection<EMAMInitialGuessSymbol> initialGuesses = SpecificationConverter.getInitialGuesses(eqs, component);
//
//            // getVariables adds only ports, if there is no specification available, consider renaming getVariables
//            for (EMAMSymbolicVariableSymbol variable : SpecificationConverter.getVariables(eqs, component)) {
//                variableIndex.put(variable.getName(), currentIndex++);
//                this.variables.add(NameHelper.replaceWithUnderScore(variable.getName()));
//
//                Optional<EMAMInitialValueSymbol> initialValue = initialValues.stream()
//                        .filter(i -> i.getNameToAccess().equals(variable.getName()))
//                        .findFirst();
//                Optional<EMAMInitialGuessSymbol> initialGuess = initialGuesses.stream()
//                        .filter(i -> i.getNameToAccess().equals(variable.getName()))
//                        .findFirst();
//
//                if (initialValue.isPresent()) {
//                    this.initialValues.add(initialValue.get().getValue().getTextualRepresentation());
//                } else if (initialGuess.isPresent()) {
//                    this.initialValues.add(initialGuess.get().getValue().getTextualRepresentation());
//                } else {
//                    this.initialValues.add("0");
//                }
//            }
//        }
//
//        massMatrixDiag = new ArrayList<>(variables.size());
//        function = new ArrayList<>(variables.size());
//        for (int i = 0; i < variables.size(); i++) {
//            massMatrixDiag.add(null);
//            function.add(null);
//        }
//        Set<Integer> usedIndeces = new HashSet<>();
//        List<EMAMEquationSymbol> algebraicEquations = new ArrayList<>();
//
//        for (EMAComponentInstanceSymbol component : eqs.getComponentInstanceSymbols()) {
//            Collection<EMAMEquationSymbol> equations = SpecificationConverter.getEquations(eqs, component);
//            if (!equations.isEmpty()) {
//                for (EMAMEquationSymbol equation : equations) {
//                    Optional<String> differentialVariable = CalculateDifferentialVariable.getDifferentialVariable(equation);
//                    if (differentialVariable.isPresent()) {
//                        Integer index = variableIndex.get(differentialVariable.get());
//                        massMatrixDiag.set(index, "1");
//                        function.set(index, getFunction(equation.getRightExpression(), variableIndex, inportMapping));
//                    } else
//                        algebraicEquations.add(equation);
//                }
//            } else {
//                algebraicComponents.add(component); // Integrate and Derivative should have been caclulated equations
//            }
//        }
//
//        currentIndex = 0;
//        for (EMAMEquationSymbol algebraicEquation : algebraicEquations) {
//            currentIndex = nextIndex(usedIndeces, currentIndex);
//            MathArithmeticExpressionSymbol minus = new MathArithmeticExpressionSymbol();
//            minus.setOperator("-");
//            minus.setLeftExpression(algebraicEquation.getLeftExpression());
//            minus.setRightExpression(new MathParenthesisExpressionSymbol(algebraicEquation.getRightExpression()));
//
//            massMatrixDiag.set(currentIndex, "0");
//            function.set(currentIndex, getFunction(minus, variableIndex, inportMapping));
//        }
//
//        for (EMAComponentInstanceSymbol algebraicComponent : algebraicComponents) {
//            Collection<EMAMSymbolicVariableSymbol> portVariables = SpecificationConverter.getVariables(eqs, algebraicComponent);
//            for (EMAMSymbolicVariableSymbol portVariable : portVariables) {
//                currentIndex = nextIndex(usedIndeces, currentIndex);
//                massMatrixDiag.set(currentIndex, "0");
//                function.set(currentIndex,
//                        String.join("-", getFunction(portVariable, variableIndex, inportMapping),
//                        CPPEquationSystemHelper.getNameOfPortOfComponent(portVariable.getPort().get())));
//            }
//        }
//
//        if (massMatrixDiag.stream().filter(s -> "1".equals(s))
//                .findFirst().isPresent())
//            this.isAlgebraic = false;
    }

    private String getFunction(MathExpressionSymbol symbol,
                               Map<String, Integer> variableIndex, Map<String, String> inportMapping) {
        MathExpressionSymbol copy = CopyEMAMMathExpressionSymbol.copy(symbol);
        NameReplacer.replaceNames(copy, s -> {
            if (variableIndex.containsKey(s)) return String.format("x[%s]", variableIndex.get(s));
            if (inportMapping.containsKey(s)) return inportMapping.get(s);
            return s;
        });
        return copy.getTextualRepresentation();
    }

    private static Integer nextIndex(Set<Integer> usedIndeces, Integer currentIndex) {
        while (usedIndeces.contains(currentIndex))
            currentIndex++;
        usedIndeces.add(currentIndex);
        return currentIndex;
    }


    private static class CalculateDifferentialVariable implements EMAMMathExpressionSymbolParentAwareVisitor {

        private Optional<String> differentialVariable = Optional.empty();

        public static Optional<String> getDifferentialVariable(EMAMEquationSymbol equation) {
            CalculateDifferentialVariable calc = new CalculateDifferentialVariable();
            calc.handle(equation);
            return calc.differentialVariable;
        }

        @Override
        public void visit(MathMatrixNameExpressionSymbol node) {
            if (node.getNameToAccess().equals(de.monticore.lang.monticar.semantics.Constants.derivativeOperatorName)) {
                if(! (getParents().peek() instanceof EMAMEquationSymbol))
                    Log.error("Not supported diff operator: should stand alone to the lift side of an equation");
                if (! ((EMAMEquationSymbol) getParents().peek()).getLeftExpression().equals(node))
                    Log.error("Not supported diff operator: should stand alone to the lift side of an equation");
                else if (node.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().size() != 1)
                    Log.error("Not supported diff operator: wrong number of arguments");
                else if (!(node.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbol(0).get()
                        instanceof MathNameExpressionSymbol))
                    Log.error("Not supported diff operator: argument should be Name");
                else
                    this.differentialVariable = Optional.ofNullable(
                            ((MathNameExpressionSymbol) node.getMathMatrixAccessOperatorSymbol()
                            .getMathMatrixAccessSymbol(0).get())
                                    .getNameToAccess());
            }
        }


        private Stack<MathExpressionSymbol> parents = new Stack<>();

        @Override
        public Stack<MathExpressionSymbol> getParents() {
            return this.parents;
        }

        private Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

        @Override
        public Set<MathExpressionSymbol> getVisitedSymbols() {
            return this.visitedSymbols;
        }
    }


    public String getName() {
        return name;
    }

    public List<String> getIncludes() {
        return includes;
    }

    public List<String> getVariableWithType() {
        return variableWithType;
    }

    public List<String> getInits() {
        return inits;
    }

    public List<String> getInports() {
        return inports;
    }

    public List<String> getMassMatrixDiag() {
        return massMatrixDiag;
    }

    public List<String> getFunctions() {
        return functions;
    }

    public boolean isAlgebraic() {
        return isAlgebraic;
    }

    public double getAtol() {
        return atol;
    }

    public double getRtol() {
        return rtol;
    }

    public double getJtol() {
        return jtol;
    }

    public double getDt_init() {
        return dt_init;
    }

    public double getDt_max() {
        return dt_max;
    }

    public double getLoggingLevel() {
        return loggingLevel;
    }
}
