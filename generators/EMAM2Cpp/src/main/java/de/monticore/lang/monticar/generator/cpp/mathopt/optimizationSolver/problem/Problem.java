/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem;

import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationType;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

/**
 * Represents an abstract optimization problem of the general form
 *
 */
public class Problem {

    /**
     * identification number for problem
     */
    private int id = this.hashCode();

    /**
     * defines whether we do minimization or maximization
     */
    private MathOptimizationType optimizationProblemType;

    // fields
    /**
     * dimension of the optimization variable
     */
    private int n;

    private List<MathValueSymbol> optimizationVariables = new ArrayList<>();
    private List<MathValueSymbol> independentOptVariables = new ArrayList<>();

    private List<MathOptimizationConditionSymbol> constraintFunctions = new ArrayList<>();

    private MathExpressionSymbol stepSize;

    /**
     * dimensions of the optimization variable
     */
    private Vector<Integer> optimizationVariableDimensions;

    /**
     * variable which contains the objective value
     */
    private MathValueSymbol objectiveValueVariable;

    /**
     * objective function
     */
    private MathExpressionSymbol objectiveFunction;

    // getter setter

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
        if (id <= 0) {
            //builtin hashcode introduces problems for code-testing
            // (each java execution may yield a different value)
            //this.id = this.hashCode();
            String objVarName = getObjectiveValueVariable().getName();
            this.id=0;
            int arr[] = objVarName.chars().toArray();
            for (int i = 0; i<arr.length;i++){
                this.id += arr[i] + i * 256;
            }
        }
    }

    public int getN() {
        return n;
    }

    public void setN(int n) {
        this.n = n;
    }

    public List<MathValueSymbol> getOptimizationVariables() {
        return optimizationVariables;
    }

    public void setOptimizationVariables(List<MathValueSymbol> optimizationVariables) {
        this.optimizationVariables = optimizationVariables;
    }

    public List<MathValueSymbol> getIndependentOptVariables() {
        return independentOptVariables;
    }

    public void setIndependentVariables(List<MathValueSymbol> independentOptVariables) {
        this.independentOptVariables = independentOptVariables;
    }

    public MathValueSymbol getObjectiveValueVariable() {
        return objectiveValueVariable;
    }

    public void setObjectiveValueVariable(MathValueSymbol objectiveValueVariable) {
        this.objectiveValueVariable = objectiveValueVariable;
    }

    public MathExpressionSymbol getObjectiveFunction() {
        return objectiveFunction;
    }

    public void setObjectiveFunction(MathExpressionSymbol objectiveFunction) {
        this.objectiveFunction = objectiveFunction;
    }

    public Vector<Integer> getOptimizationVariableDimensions() {
        return optimizationVariableDimensions;
    }

    public void setOptimizationVariableDimensions(Vector<Integer> optimizationVariableDimensions) {
        this.optimizationVariableDimensions = optimizationVariableDimensions;
    }

    public MathOptimizationType getOptimizationProblemType() {
        return optimizationProblemType;
    }

    public void setOptimizationProblemType(MathOptimizationType optimizationProblemType) {
        this.optimizationProblemType = optimizationProblemType;
    }

    public MathExpressionSymbol getStepSize() {
        return stepSize;
    }

    public void setStepSize(MathExpressionSymbol stepSize) {
        this.stepSize = stepSize;
    }

    /**
     * Default value if no lower bound is set
     */
    public final static String LOWER_BOUND_INF = "-1E19";
    /**
     * Default value if no upper bound is set
     */
    public final static String UPPER_BOUND_INF = "1E19";
    /**
     * number of constraints in function g
     */
    private int m;

    /**
     * lower bound of x
     */
    private Vector<String> xL = new Vector<>();

    /**
     * upper bound of x
     */
    private Vector<String> xU = new Vector<>();

    /**
     * lower bound of g
     */
    private Vector<String> gL = new Vector<>();

    /**
     * upper bound of g
     */
    private Vector<String> gU = new Vector<>();

    /**
     * Additional constraints on optimization variable
     */
    private Vector<String> xMatrixElementConstraints = new Vector<>();

    // getter setter

    public int getM() {
        return m;
    }

    public void setM(int m) {
        this.m = m;
    }

    public List<MathOptimizationConditionSymbol> getConstraintFunctions() {
        return constraintFunctions;
    }

    public void setConstraintFunctions(List<MathOptimizationConditionSymbol> constraintFunctions) {
        this.constraintFunctions = constraintFunctions;
    }


    public Vector<String> getxL() {
        return xL;
    }

    public void setxL(Vector<String> xL) {
        this.xL = xL;
    }

    public Vector<String> getxU() {
        return xU;
    }

    public void setxU(Vector<String> xU) {
        this.xU = xU;
    }

    public Vector<String> getgL() {
        return gL;
    }

    public void setgL(Vector<String> gL) {
        this.gL = gL;
    }

    public Vector<String> getgU() {
        return gU;
    }

    public void setgU(Vector<String> gU) {
        this.gU = gU;
    }

    public void setXMatrixElementConstraints(Vector<String> xMatrixElementConstraints) {
        this.xMatrixElementConstraints = xMatrixElementConstraints;
    }

    public Vector<String> getXMatrixElementConstraints() {
        return xMatrixElementConstraints;
    }
}
