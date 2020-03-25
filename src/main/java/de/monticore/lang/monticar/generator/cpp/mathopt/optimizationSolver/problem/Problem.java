/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem;

import de.monticore.lang.mathopt._symboltable.MathOptimizationType;

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

    /**
     * name of the optimization variable
     */
    private String optimizationVariableName;

    /**
     * data type of the optimization variable
     */
    private String optimizationVariableType;

    /**
     * dimensions of the optimization variable
     */
    private Vector<Integer> optimizationVariableDimensions;

    /**
     * variable which contains the objective value
     */
    private String objectiveValueVariable;

    /**
     * objective function
     */
    private String objectiveFunction;

    // getter setter

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
        if (id <= 0) {
            this.id = this.hashCode();
        }
    }

    public int getN() {
        return n;
    }

    public void setN(int n) {
        this.n = n;
    }

    public String getOptimizationVariableName() {
        return optimizationVariableName;
    }

    public void setOptimizationVariableName(String optimizationVariableName) {
        this.optimizationVariableName = optimizationVariableName;
    }

    public String getObjectiveValueVariable() {
        return objectiveValueVariable;
    }

    public void setObjectiveValueVariable(String objectiveValueVariable) {
        this.objectiveValueVariable = objectiveValueVariable;
    }

    public String getObjectiveFunction() {
        return objectiveFunction;
    }

    public void setObjectiveFunction(String objectiveFunction) {
        this.objectiveFunction = objectiveFunction;
    }

    public String getOptimizationVariableType() {
        return optimizationVariableType;
    }

    public void setOptimizationVariableType(String optimizationVariableType) {
        this.optimizationVariableType = optimizationVariableType;
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
     * function g: R^n -> R^m
     */
    private Vector<String> constraintFunctions = new Vector<>();
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

    public Vector<String> getConstraintFunctions() {
        return constraintFunctions;
    }

    public void setConstraintFunctions(Vector<String> constraintFunctions) {
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
