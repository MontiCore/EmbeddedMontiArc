/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.template;


import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationType;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.*;

public abstract class SolverViewModel extends ViewModelBase {

    private String id;

    /* Variables which must be set in constructor */

    private MathOptimizationType optimizationProblemType;

    private MathExpressionSymbol stepSize;

    private MathValueSymbol objectiveVariable;
    private MathExpressionSymbol objectiveFunction;

    private List<MathValueSymbol> optimizationVariables;
    private List<MathValueSymbol> independentVariables;

    private List<MathOptimizationConditionSymbol> constraintFunctions;

    //additional constraints on single optimization variable matrix elements which can not be evaluated static
    private Vector<String> xMatrixElementConstraints; //Rewrite?

    /* Variables not contained in Problem class, set after object creation */
    private String callSolverName;
    private EMAMBluePrintCPP mathBluePrint;
    private HashSet<String> options = new LinkedHashSet<>();

    /* Derived Variables / getters */
        //private Vector<Integer> optimizationVariableDimensions;
        //private int numberOptimizationVariables;
        //private int numberIndependentVariables;
        //private int numberConstraints;

    private List<MathValueSymbol> emamVariables = new ArrayList<>();

    /* ?? */
    /**
     * For automatic differentiation a active type is required
     */
    private String optimizationVariableTypeActive;

    /**
     * Lower bound for x
     */
    private Vector<String> xL;

    /**
     * Upper bound for x
     */
    private Vector<String> xU;

    /**
     * Lower bound for g(x)
     */
    private Vector<String> gL;

    /**
     * Upper bound for g(x)
     */
    private Vector<String> gU;

    /**
     * starting point of the iteration for x
     */
    private Vector<Double> initX;




    // constructor

    /**
     * Generated a generator view model from a optimization problem
     *
     * @param problem non linear optimization problem
     */
    public SolverViewModel(Problem problem) {
        this.optimizationProblemType = problem.getOptimizationProblemType();
        //this.numberVariables = problem.getN();
        //this.numberConstraints = problem.getM();
        this.optimizationVariables = problem.getOptimizationVariables();

        this.independentVariables = problem.getIndependentOptVariables();
        this.constraintFunctions = problem.getConstraintFunctions();
        this.objectiveVariable = problem.getObjectiveValueVariable();
        this.objectiveFunction = problem.getObjectiveFunction();

        this.stepSize = problem.getStepSize();

        //legacy
        this.gL = problem.getgL();
        this.gU = problem.getgU();
        this.xL = problem.getxL();
        this.xU = problem.getxU();
        this.xMatrixElementConstraints = problem.getXMatrixElementConstraints();

        //this.optimizationVariableDimensions = problem.getOptimizationVariableDimensions();
        this.id = Integer.toString(problem.getId());

        this.initX = calculateInitialX();
    }

    /* Get methods for Problem associated variables */

    public MathValueSymbol getObjectiveVariable() { return objectiveVariable; }

    public MathExpressionSymbol getObjectiveFunction() { return objectiveFunction; }

    public List<MathValueSymbol> getOptimizationVariables() { return optimizationVariables; }

    public List<MathValueSymbol> getIndependentVariables() { return independentVariables; }

    public List<MathOptimizationConditionSymbol> getConstraintFunctions() { return constraintFunctions;  }

    public List<MathValueSymbol> getEmamVariables() { return emamVariables; }

    public String getId() {
        return id;
    }

    public MathExpressionSymbol getStepSize() { return stepSize; }

    /* Get and Set methods for other variables */

    public String getCallSolverName() { return callSolverName; }
    public void setCallSolverName(String callSolverName) { this.callSolverName = callSolverName; }


    /* Derived values */

    public Boolean hasObjectiveVariable() {
        return objectiveVariable != null;
    }

    public String getObjectiveVariableName() {
        if(objectiveVariable != null) {
            return objectiveVariable.getName();
        }else{
            return "y";
        }
    }

    public int getNumberIndependentVariables(){
        return independentVariables.size();
    }

    public int getNumberOptimizationVariables(){ return optimizationVariables.size(); }

    public int getNumberVariables() {
        return getNumberIndependentVariables() + getNumberOptimizationVariables();
    }

    public int getNumberConstraints() {
        return constraintFunctions.size();
    }

    public String getObjectiveFunctionStr(){
        return this.objectiveFunction.getTextualRepresentation();
    }

    /**
     * Calculates a starting point for variable x.
     * Calculation is done by taking the middle between xL and xU
     *
     * @return initial starting point for x
     */
    protected Vector<Double> calculateInitialX() {
        Vector<Double> initX = new Vector<>(getNumberVariables());
        for (int i = 0; i < getNumberVariables(); i++) {
            initX.add((double) (-100 + (int) (Math.random() * 100)));
        }
        return initX;
    }

    protected void replaceVariable(String var, String replacementVar) {
        // Necessary?
        /*
        objectiveFunction = replaceVariableInExpr(objectiveFunction, var, replacementVar);
        for (int i = 0; i < constraintFunctions.size(); i++) {
            constraintFunctions.set(i, replaceVariableInExpr(constraintFunctions.get(i), var, replacementVar));
        }*/
    }


    public String getOptimizationVariableTypeActive() {
        return optimizationVariableTypeActive;
    }

    //Obsolete, Template engine already incorporates indexes
    public void setKnownVariablesWithNumbers() {
        /*List<String>  knownVariables = this.getKnownVariablesAsArray();
        for (int i = 0; i < knownVariables.size(); i++){
            knownVariablesWithNumbers.add(new NumberedVariable(knownVariables.get(i),i));
        }*/
    }

    public MathOptimizationType getOptimizationProblemType() {
        return optimizationProblemType;
    }

    public void setOptimizationProblemType(MathOptimizationType optimizationProblemType) {
        this.optimizationProblemType = optimizationProblemType;
    }

    public HashSet<String> getOptions() {
        return options;
    }

    public void setOptions(HashMap<String, String> map) {
        options.clear();
        for (String key : map.keySet())
            options.add(String.format("%s    %s", key, map.get(key)));
    }

    public void setOptimizationVariableTypeActive(String optimizationVariableTypeActive) {
        this.optimizationVariableTypeActive = optimizationVariableTypeActive;
    }

    public Vector<String> getxMatrixElementConstraints() {
        return xMatrixElementConstraints;
    }

    public void setxMatrixElementConstraints(Vector<String> xMatrixElementConstraints) {
        this.xMatrixElementConstraints = xMatrixElementConstraints;
    }
}
