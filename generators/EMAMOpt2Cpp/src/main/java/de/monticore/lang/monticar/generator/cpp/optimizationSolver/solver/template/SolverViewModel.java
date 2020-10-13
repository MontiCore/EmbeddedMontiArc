/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver.template;

import de.monticore.lang.mathopt._symboltable.MathOptimizationType;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;

import java.util.*;

public abstract class SolverViewModel extends ViewModelBase {

    private String id;

    /**
     * Name of the generated solver execution class name
     */
    private String callSolverName;

    private MathOptimizationType optimizationProblemType;

    /**
     * Name of the optimization variable
     */
    private String optimizationVariableName;

    /**
     * Data type of the optimization variable
     */
    private String optimizationVariableType;

    /**
     * Dimensions of the optimization variable
     */
    private Vector<Integer> optimizationVariableDimensions;

    /**
     * For automatic differentiation a active type is required
     */
    private String optimizationVariableTypeActive;

    /**
     * Name of the final objective value variable
     */
    private String objectiveVariableName;

    /**
     * Number of variables from x
     */
    private int numberVariables;

    /**
     * Number of constraints
     */
    private int numberConstraints;

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
     * additional constraints on single optimization variable matrix elements which can not be evaluated static
     */
    private Vector<String> xMatrixElementConstraints;

    /**
     * starting point of the iteration for x
     */
    private Vector<Double> initX;

    /**
     * objective function, x and obj_value are resavated for optimization variable and objective value
     */
    private String objectiveFunction;

    /**
     * list of constraints
     */
    private Vector<String> constraintFunctions;

    /**
     * all variables from EMAM workspace
     */
    private HashSet<String> knownVariables = new LinkedHashSet<>();

    /**
     * all variables with declaration type from EMAM workspace
     */
    private HashSet<String> knownVariablesWithType = new LinkedHashSet<>();

    private HashSet<String> options = new LinkedHashSet<>();

    // constructor

    /**
     * Generated a generator view model from a optimization problem
     *
     * @param problem non linear optimization problem
     */
    public SolverViewModel(Problem problem) {
        this.optimizationProblemType = problem.getOptimizationProblemType();
        this.numberVariables = problem.getN();
        this.numberConstraints = problem.getM();
        this.constraintFunctions = problem.getConstraintFunctions();
        this.gL = problem.getgL();
        this.gU = problem.getgU();
        this.xL = problem.getxL();
        this.xU = problem.getxU();
        this.xMatrixElementConstraints = problem.getXMatrixElementConstraints();
        this.objectiveFunction = problem.getObjectiveFunction();
        this.optimizationVariableName = problem.getOptimizationVariableName();
        setOptimizationVariableType(problem.getOptimizationVariableType());
        this.objectiveVariableName = problem.getObjectiveValueVariable();
        this.optimizationVariableDimensions = problem.getOptimizationVariableDimensions();
        id = Integer.toString(problem.getId());

        this.initX = calculateInitialX();
    }

    // getter setter methods

    private static String replaceVariableInExpr(String expr, String var, String replacementVar) {
        String result = expr;

        String sepVar1 = "" + var + "";
        String sepVar2 = "" + var + "[";

        String sepRepVar1 = "" + replacementVar + "";
        String sepRepVar2 = "" + replacementVar + "[";

        result = result.replace(sepVar1, sepRepVar1);
        result = result.replace(sepVar2, sepRepVar2);
        return result;
    }

    public String getCallSolverName() {
        return callSolverName;
    }

    public void setCallSolverName(String callSolverName) {
        this.callSolverName = callSolverName;
    }

    public String getOptimizationVariableName() {
        return optimizationVariableName;
    }

    public void setOptimizationVariableName(String optimizationVariableName) {
        this.optimizationVariableName = optimizationVariableName;
    }

    public String getObjectiveVariableName() {
        return objectiveVariableName;
    }

    public void setObjectiveVariableName(String objectiveVariableName) {
        this.objectiveVariableName = objectiveVariableName;
    }

    public int getNumberVariables() {
        return numberVariables;
    }

    public void setNumberVariables(int numberVariables) {
        this.numberVariables = numberVariables;
    }

    public int getNumberConstraints() {
        return numberConstraints;
    }

    public void setNumberConstraints(int numberConstraints) {
        this.numberConstraints = numberConstraints;
    }

    public List<String> getxL() {
        return xL;
    }

    public void setxL(Vector<String> xL) {
        this.xL = xL;
    }

    public List<String> getxU() {
        return xU;
    }

    public void setxU(Vector<String> xU) {
        this.xU = xU;
    }

    public List<String> getgL() {
        return gL;
    }

    public void setgL(Vector<String> gL) {
        this.gL = gL;
    }

    public List<String> getgU() {
        return gU;
    }

    public void setgU(Vector<String> gU) {
        this.gU = gU;
    }

    public List<Double> getInitX() {
        return initX;
    }

    public void setInitX(Vector<Double> initX) {
        this.initX = initX;
    }

    public String getObjectiveFunction() {
        return objectiveFunction;
    }

    public void setObjectiveFunction(String objectiveFunction) {
        this.objectiveFunction = objectiveFunction;
    }

    public Vector<String> getConstraintFunctions() {
        return constraintFunctions;
    }

    public void setConstraintFunctions(Vector<String> constraintFunctions) {
        this.constraintFunctions = constraintFunctions;
    }

    public String getOptimizationVariableType() {
        return optimizationVariableType;
    }

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    // methods

    public void setOptimizationVariableType(String optimizationVariableType) {
        this.optimizationVariableType = optimizationVariableType;
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
        objectiveFunction = replaceVariableInExpr(objectiveFunction, var, replacementVar);
        for (int i = 0; i < constraintFunctions.size(); i++) {
            constraintFunctions.set(i, replaceVariableInExpr(constraintFunctions.get(i), var, replacementVar));
        }
    }

    protected boolean containsVariable(String variable) {
        Boolean result = false;
        if (exprContainsVar(objectiveFunction, variable)) {
            result = true;
        }
        for (String s : constraintFunctions) {
            if (exprContainsVar(s, variable)) {
                result = true;
            }
        }
        return result;
    }

    private boolean exprContainsVar(String expr, String variable) {

        String sepVar1 = "" + variable + "";
        String sepVar2 = "" + variable + "[";

        Boolean matches1 = expr.contains(sepVar1);
        Boolean matches2 = expr.contains(sepVar2);

        return matches1 || matches2;
    }

    protected String findRelpacementVariable(String variable) {
        String replacementVar = variable;
        while (containsVariable(replacementVar)) {
            replacementVar += "Tmp";
        }
        return replacementVar;
    }

    public String getOptimizationVariableTypeActive() {
        return optimizationVariableTypeActive;
    }

    public HashSet<String> getKnownVariables() {
        return knownVariables;
    }

    public HashSet<String> getKnownVariablesWithType() {
        return knownVariablesWithType;
    }

    public void setKnownVariablesFromBluePrint(BluePrintCPP bluePrint) {
        List<Variable> variables = bluePrint.getMathInformationRegister().getVariables();
        variables.addAll(bluePrint.getVariables());
        for (Variable v : variables) {
            String name = v.getNameTargetLanguageFormat();
            String type = v.getVariableType().getTypeNameTargetLanguage();
            if ((!name.contentEquals(optimizationVariableName)) && (!name.contentEquals(objectiveVariableName)) && (!v.isForLoopVariable()) && (!knownVariables.contains(name))) {
                knownVariables.add(name);
                knownVariablesWithType.add(String.format("%s %s", type, name));
            }
        }
    }

    public Vector<Integer> getOptimizationVariableDimensions() {
        return optimizationVariableDimensions;
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
