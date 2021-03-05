/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.ipopt;

import de.monticore.lang.math._ast.ASTMathAssignmentDeclarationStatement;
import de.monticore.lang.math._matrixprops.MatrixProperties;
import de.monticore.lang.math._symboltable.JSValue;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolVisitor;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.problem.Problem;
import de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.template.SolverViewModel;

import java.util.*;
import java.util.stream.Collectors;

import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.ObjectUtils;
import org.jscience.mathematics.number.Rational;

/**
 * Contains all necessary information needed to generate a freemarker template of IPOPT C++ code.
 *
 */
public class IpoptViewModel extends SolverViewModel {

    /**
     * Reservated in IPOPT calculations as optimization variable
     */
    private static final String IPOPT_OPTIMIZATION_VAR = "x";

    /**
     * Reservated in IPOPT calculations as objective variable
     */
    private static final String IPOPT_OBJECTIVE_VAR = "fg";

    /**
     * Reservated in IPOPT calculations as constraint function pointer
     */
    private static final String IPOPT_CONSTRAINT_FUNCTION_VAR = "fg";

    /**
     * array of all reservated variables used by IPOPT
     */
    private static final String[] IPOPT_RESERVATED_VARS = {IPOPT_OPTIMIZATION_VAR, IPOPT_OBJECTIVE_VAR, IPOPT_CONSTRAINT_FUNCTION_VAR};

    // constructor

    /**
     * Generated a IPOPT view model from a optimization problem
     *
     * @param problem non linear optimization problem
     */
    public IpoptViewModel(Problem problem) {
        super(problem);
        this.setCallSolverName("CallIpopt" + problem.getId());
    }


    public Boolean isMPCModel(){
        Boolean containsAssignment = false;

        //ToDo: Better check if recursive definition exists. aka. same active var left and right.
        for( MathOptimizationConditionSymbol optConSym : getConstraintFunctions()){
            if(optConSym.getOperator()=="==")
                containsAssignment = true;
        }
        return hasStepSize() && containsAssignment;
    }

    public String getVariableType(MathValueSymbol symbol){
        //String debug = "type"+symbol.getType().getName() +
        //        "\n typeType: "+symbol.getType().getType().getName() + "\n isRational: "+symbol.getType().getType().isRational();
        String result = "";
        String varTypeStr = symbol.getType().getType().getName();
        if (varTypeStr.contentEquals("Q"))
            result = "AD<double>";
        if(symbol.getType().getDimensions().size() > 0)
            result = "ADMat";
        return result;
    }

    public String getIpoptVarOffset(MathValueSymbol symbol){
        int var = -1;
        int pre_offset = 0;
        if(getIndependentVariables().indexOf(symbol) != -1) {
            var = getIndependentVariables().indexOf(symbol);
            pre_offset = getNumberOptimizationVariables() * getStepSizeCount();
        }
        if(getOptimizationVariables().indexOf(symbol) != -1) {
            var = getOptimizationVariables().indexOf(symbol);
        }
        return pre_offset+" + " + getStepSizeCount() * var; //+ " * " + var;
    }

    public String getIpoptVarRef(MathValueSymbol symbol){return "_V_"+symbol.getName();}

    public String getIpoptVarRef(String symbolName){return "_V_"+symbolName;}

    public String replaceVariablesWithIpoptVectorEntry(String code){
        String result = "";
        for (MathValueSymbol var : getOptimizationVariables()){
            code.replace(var.getName(), getIpoptVarRef(var));
        }
        for (MathValueSymbol var : getIndependentVariables()){
            //code.replace(var.getName(), getIpoptIndex( var));
        }
        return result;
    }

    //ToDo: Move to SolverViewModel
    public boolean hasStepSize(){
        if(getStepSize()!= null)
            return true;
        else
            return false;
    }

    public MathMatrixVectorExpressionSymbol getStepSizeVectorSymbol(){
        MathMatrixVectorExpressionSymbol result = null;
        MathExpressionSymbol mathExpression = getStepSize();
        //ToDo: Add CoCo to ensure this always holds.
        if(mathExpression.isAssignmentExpression()) {
            MathAssignmentExpressionSymbol assignExpression = (MathAssignmentExpressionSymbol) mathExpression;
            MathExpressionSymbol assignChildExpression = assignExpression.getExpressionSymbol();

            result = (MathMatrixVectorExpressionSymbol) assignChildExpression;
        }
        return result;
    }

    public String getStepSizeName(){
        String result = "";
        MathExpressionSymbol mathExpression = getStepSize();
        //ToDo: Add CoCo to ensure this always holds.
        if(mathExpression.isAssignmentExpression()) {
            MathAssignmentExpressionSymbol assignExpression = (MathAssignmentExpressionSymbol) mathExpression;

            result = assignExpression.getNameOfMathValue();
        }
        return result;
    }


    public int getStepSizeMin(){
        String result = "";
        MathMatrixVectorExpressionSymbol vectorExpressionSymbol = getStepSizeVectorSymbol();
        MathNumberExpressionSymbol startExpression = (MathNumberExpressionSymbol) vectorExpressionSymbol.getStart();
        return startExpression.getValue().getRealNumber().intValue();
    }

    public int getStepSizeMax(){
        String result = "";
        MathMatrixVectorExpressionSymbol vectorExpressionSymbol = getStepSizeVectorSymbol();
        MathNumberExpressionSymbol endExpression = (MathNumberExpressionSymbol) vectorExpressionSymbol.getEnd();
        return endExpression.getValue().getRealNumber().intValue();
    }

    public int getStepSizeCount(){
        return (getStepSizeMax()-getStepSizeMin()+1);
    }

    public String getVariableInitialization(MathValueSymbol variable){
        String value = "0";
        if(variable.isValueExpression()){
            //value = variable.getValue().getTextualRepresentation();
            MathExpressionSymbol sym = variable.getValue();
            if(sym != null) {
                MathNameExpressionSymbol mneSym = (MathNameExpressionSymbol) sym;
                value = mneSym.getTextualRepresentation();
            }
        }
        return value;
    }

    public String getVariableUpperBound(MathValueSymbol variable){
        double result = 1E19;
        if(variable.getType().getType().isPresentRange()) {
            result = variable.getType().getType().getRange().getStartValue().doubleValue();
        }
        return Double.toString(result);
    }

    public String getVariableLowerBound(MathValueSymbol variable){
        double result = -1E19;
        if(variable.getType().getType().isPresentRange()) {
            result = variable.getType().getType().getRange().getEndValue().doubleValue();
        }
        return Double.toString(result);
    }

    public String getConstraintUpperBound(MathOptimizationConditionSymbol constraint){
        if(constraint.getUpperBound().isPresent())
            return constraint.getUpperBound().get().getTextualRepresentation();
        else
            return "1E19";
    }

    public String getConstraintLowerBound(MathOptimizationConditionSymbol constraint){
        if(constraint.getLowerBound().isPresent())
            return constraint.getLowerBound().get().getTextualRepresentation();
        else
            return "-1E19";
    }

    public String listClassesInScope(){
        String result = "";

        for ( Collection<Symbol> x : getIndependentVariables().get(0).getEnclosingScope().getLocalSymbols().values()){
            result += ","+x.getClass().toString();
            for(Symbol s : x){
                result += ", "+s.getClass().toString();
            }
        }
        return result;
    }

    public String transformEMAMMatrixAccessToVectorNotation(MathMatrixNameExpressionSymbol symbol, String VectorName, Integer indexOffset){
        String result = "";
        String MatrixName = symbol.getNameToAccess();
        String MatrixIndex = symbol.getMathMatrixAccessOperatorSymbol().getTextualRepresentation();

        result = VectorName + "[" + getIpoptVarRef(MatrixName)+" + "+ MatrixIndex + "]";

        return result;
    }


    public boolean isOptScopedVariable(String varName){
        for(MathValueSymbol var : getIndependentVariables()){
            if(var.getName().equals(varName))
                return true;
        }
        for(MathValueSymbol var : getOptimizationVariables()){
            if(var.getName().equals(varName))
                return true;
        }
        return false;
    }

    public boolean containsOptScopedVariable(MathExpressionSymbol symbol){
        //Test for MathValueSymbols and MathMatrixAccessors
        MathValueVisitor valueVisitor = new MathValueVisitor();
        MathAccessVisitor accessVisitor = new MathAccessVisitor();

        valueVisitor.handle(symbol);
        accessVisitor.handle(symbol);

        for (MathValueSymbol value : valueVisitor.getMathValueSymbols()){
            if(isOptScopedVariable(value.getName()))
                return true;
        }

        for (MathMatrixNameExpressionSymbol value : accessVisitor.getMathMatrixNameExpressionSymbols()){
            if(isOptScopedVariable(value.getNameToAccess()))
                return true;
        }
        return false;
    }


    public MathOptimizationConditionSymbol resolveExpression(){
        return null;
    }

    /* This function generates a new constraint list. It adds constraints if:
    *  - active variables are
    *
    * */
    public List<MathOptimizationConditionSymbol> getSimplifiedConstraintFunctions(){

            List<MathOptimizationConditionSymbol> constraints = getConstraintFunctions();
            List<MathOptimizationConditionSymbol> modifiedConstraints = new ArrayList<>();

            for(MathOptimizationConditionSymbol constraint : constraints) {
                boolean handledConstraint = false;
                //Initialize visitors
/*                MathValueVisitor mvvLower = new MathValueVisitor();
                MathValueVisitor mvvExpression = new MathValueVisitor();
                MathValueVisitor mvvUpper = new MathValueVisitor();
                MathAccessVisitor mavLower = new MathAccessVisitor();
                MathAccessVisitor mavExpression = new MathAccessVisitor();
                MathAccessVisitor mavUpper = new MathAccessVisitor();

                mvvLower.handle(constraint.getLowerBound().get());
                mavLower.handle(constraint.getLowerBound().get());
                mvvUpper.handle(constraint.getUpperBound().get());
                mavUpper.handle(constraint.getUpperBound().get());
                mvvExpression.handle(constraint.getBoundedExpression());
                mavExpression.handle(constraint.getBoundedExpression());

                List<MathValueSymbol> lowerMathSymbols = mvvLower.getMathValueSymbols();
                List<MathValueSymbol> upperMathSymbols = mvvUpper.getMathValueSymbols();
                List<MathValueSymbol> expreMathSymbols = mvvExpression.getMathValueSymbols();

                // Get used MathValueSymbols and filter for variables declared in optimization scope.
              List<MathValueSymbol> lowerMathSymbols = mvvLower.getMathValueSymbols().stream().
                        filter(mathSymbol -> isOptScopedVariable(mathSymbol.getName())).collect(Collectors.toList());
                List<MathValueSymbol> upperMathSymbols = mvvUpper.getMathValueSymbols().stream().
                        filter(mathSymbol -> isOptScopedVariable(mathSymbol.getName())).collect(Collectors.toList());
                List<MathValueSymbol> expreMathSymbols = mvvExpression.getMathValueSymbols().stream().
                        filter(mathSymbol -> isOptScopedVariable(mathSymbol.getName())).collect(Collectors.toList());*/

                //General form:     (lower <= expression <= upper)
                // We can expect expression to contain variables

                boolean lowerBoundContainsActiveVariables = false;
                if(constraint.getLowerBound().isPresent())
                    lowerBoundContainsActiveVariables = containsOptScopedVariable(constraint.getLowerBound().get());

                boolean upperBoundContainsActiveVariables = false;
                if(constraint.getUpperBound().isPresent())
                    upperBoundContainsActiveVariables = containsOptScopedVariable(constraint.getUpperBound().get());

                boolean expressionContainsActiveVariables = containsOptScopedVariable(constraint.getBoundedExpression());

                //ToDo: Add () to changed expressions

                if (lowerBoundContainsActiveVariables) {
                    // Both lower and expression contain variables defined in optimization scope.
                    // lower <= expression -->  expression - lower >= 0
                    MathArithmeticExpressionSymbol maes = new MathArithmeticExpressionSymbol();
                    maes.setOperator("-");
                    maes.setLeftExpression(constraint.getBoundedExpression());
                    maes.setRightExpression(constraint.getLowerBound().get());

                    MathNumberExpressionSymbol mnes = new MathNumberExpressionSymbol();
                    mnes.setValue(new JSValue(Rational.ZERO));

                    MathOptimizationConditionSymbol mocs = new MathOptimizationConditionSymbol(mnes, "<=", maes);
                    modifiedConstraints.add(mocs);
                    Log.warn("Constraint <"+constraint.getTextualRepresentation()+"> changed to <"+mocs.getTextualRepresentation()+".");
                    handledConstraint = true;
                }
                if (upperBoundContainsActiveVariables) {
                    //Both expression and upper contain variables defined in optimization scope.
                    // expression <= upper -->  upper - expression >= 0
                    MathArithmeticExpressionSymbol maes = new MathArithmeticExpressionSymbol();
                    maes.setOperator("-");
                    maes.setLeftExpression(constraint.getUpperBound().get());
                    maes.setRightExpression(constraint.getBoundedExpression());

                    MathNumberExpressionSymbol mnes = new MathNumberExpressionSymbol();
                    mnes.setValue(new JSValue(Rational.ZERO));

                    MathOptimizationConditionSymbol mocs = new MathOptimizationConditionSymbol(mnes, "<=", maes);
                    modifiedConstraints.add(mocs);
                    Log.warn("Constraint <"+constraint.getTextualRepresentation()+"> changed to <"+mocs.getTextualRepresentation()+".");
                    handledConstraint = true;
                }
                if (lowerBoundContainsActiveVariables && upperBoundContainsActiveVariables) {
                    //if both upper and lower contain variables, a third bound is necessary (transitivity)
                    // lower <= expression <= upper --> lower <= upper --> upper - lower >= 0
                    MathArithmeticExpressionSymbol maes = new MathArithmeticExpressionSymbol();
                    maes.setOperator("-");
                    maes.setLeftExpression(constraint.getUpperBound().get());
                    maes.setRightExpression(constraint.getLowerBound().get());

                    MathNumberExpressionSymbol mnes = new MathNumberExpressionSymbol();
                    mnes.setValue(new JSValue(Rational.ZERO));

                    MathOptimizationConditionSymbol mocs = new MathOptimizationConditionSymbol(mnes, "<=", maes);
                    modifiedConstraints.add(mocs);
                    Log.warn("Constraint <"+constraint.getTextualRepresentation()+"> changed to <"+mocs.getTextualRepresentation()+".");
                    handledConstraint = true;
                }
                if (!upperBoundContainsActiveVariables && !lowerBoundContainsActiveVariables && expressionContainsActiveVariables) {
                    //If only BoundedExpression contains variable defined in opt scope, let it through unchanged.
                    modifiedConstraints.add(constraint);
                    Log.warn("Constraint <"+constraint.getTextualRepresentation()+"> passed through.");
                    handledConstraint = true;
                }
                if(!handledConstraint){
                    Log.warn("Constraint <"+constraint.getTextualRepresentation()+"> contains no active variables and is ignored.");
                }
            }
            return modifiedConstraints;
    }

    public String getConstraintForFG_Eval(MathExpressionSymbol constraint){

        //ToDo: Check bounds (don't exceed array limits)
        String debug = "";
        String result="";

        //ToDo: isOptimizationCondition does not exist in MathExpressionSymbol
        if(constraint instanceof MathOptimizationConditionSymbol) {

            MathOptimizationConditionSymbol optCondition = (MathOptimizationConditionSymbol) constraint;

            if(optCondition.getOperator().equals("==")) {
                debug += "//"+optCondition.getTextualRepresentation()+"\nLeft: "+optCondition.getLeft().getClass().toString() + " / ";
                debug += "\nRight: "+optCondition.getRight().getClass().toString() + " / ";
                if(optCondition.getLeft() instanceof MathMatrixNameExpressionSymbol) {
                    //ToDo: Distinguish between MPC and Matrix operation

                    //MPC
                    MathMatrixNameExpressionSymbol left = (MathMatrixNameExpressionSymbol) optCondition.getLeft();
                    String leftSide = transformEMAMMatrixAccessToVectorNotation(left,"fg",1);

                    MathAccessVisitor mav = new MathAccessVisitor();
                    mav.handle(optCondition.getRight());

                    List<String[]> replacementList = new ArrayList<>();

                    for (MathMatrixNameExpressionSymbol matAccess : mav.getMathMatrixNameExpressionSymbols()){
                        //Polymorphism for the win! - Sadly not :(
                        //IpoptMathMatrixNameExpressionSymbol imneSymbol = (IpoptMathMatrixNameExpressionSymbol) matAccess;
                        //ToDo: add Vectorname as attribute with getters & setters for consistency
                        //Todo: Check if it contains a used mathValueSymbol
                        if(isOptScopedVariable(matAccess.getNameToAccess())) {
                            String replacement = transformEMAMMatrixAccessToVectorNotation(matAccess, "vars", 1);
                            String search = matAccess.getTextualRepresentation();
                            String replacementPair[] = {search, replacement};
                            replacementList.add(replacementPair);
                            //debug += "\n search: " + search + " replacement: " + replacement;
                        }
                    }

                    String rightSide = optCondition.getRight().getTextualRepresentation();
                    for(String replacementPair[] : replacementList) {
                        rightSide = rightSide.replace(replacementPair[0],replacementPair[1]);
                    }
                    result = leftSide+" = "+rightSide;
                }
            }
        }
        return result;
    }


    //ToDo: Write generic Visitor/Inspector and move it to MontiMath?
    private class MathAccessVisitor implements MathOptExpressionSymbolVisitor{

        private List<MathMatrixNameExpressionSymbol> mathMatrixNameExpressionSymbols = new ArrayList<>();
        Set<MathExpressionSymbol> visited = new HashSet<>();

        public MathAccessVisitor() {

        }

        @Override
        public Set<MathExpressionSymbol> getVisitedSymbols() {
            return visited;
        }

        @Override
        public void visit(MathMatrixNameExpressionSymbol node) {
            mathMatrixNameExpressionSymbols.add(node);
        }

        public List<MathMatrixNameExpressionSymbol> getMathMatrixNameExpressionSymbols() {
            return mathMatrixNameExpressionSymbols;
        }

    }

    private class MathValueVisitor implements MathOptExpressionSymbolVisitor{

        private List<MathValueSymbol> mathValueSymbols = new ArrayList<>();
        Set<MathExpressionSymbol> visited = new HashSet<>();

        public MathValueVisitor() {

        }

        @Override
        public Set<MathExpressionSymbol> getVisitedSymbols() {
            return visited;
        }

        @Override
        public void visit(MathValueSymbol node) {
            mathValueSymbols.add(node);
        }

        public List<MathValueSymbol> getMathValueSymbols() {
            return mathValueSymbols;
        }

    }

    /*
    private class Inspector<T> implements MathExpressionSymbolVisitor{
        private List<T> inspectedSymbols = new ArrayList<>();
        private Set<MathExpressionSymbol> visited = new HashSet<>();
        @Override
        public Set<MathExpressionSymbol> getVisitedSymbols() {
            return visited;
        }

        @Override
        public void visit(T node) {
            inspectedSymbols.add(node);
        }
    }*/

}
