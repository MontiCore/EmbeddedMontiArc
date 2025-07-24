/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.mathopt.optimizationSolver.solver.ipopt;

import de.monticore.lang.math._ast.ASTMathAssignmentDeclarationStatement;
import de.monticore.lang.math._matrixprops.MatrixProperties;
import de.monticore.lang.math._symboltable.JSValue;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathValue;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolVisitor;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
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

    //external variables from EMAM workspace, somehow derived with Blueprint
    List<Variable> externalVariables = new ArrayList<>();

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

        //Better check if recursive definition exists. aka. same active var left and right.
        for( MathOptimizationConditionSymbol optConSym : getConstraintFunctions()){
            if(optConSym.getOperator()=="==")
                containsAssignment = true;
        }
        return hasStepSize() && containsAssignment;
    }

    public boolean isExternalVariable(String varName){
        for(Variable v: getExternalVariables()){
            if(v.getName().equals(varName))
                return true;
        }

        return false;
    }

    public void setExternalVariablesFromBluePrint(EMAMBluePrintCPP bluePrint) {
        this.externalVariables.clear();
        List<Variable> variables = bluePrint.getMathInformationRegister().getVariables();
        variables.addAll(bluePrint.getVariables());
        for (Variable v : variables) {
            if (!isOptScopedVariable(v.getName())){// && (v.isConstantVariable() || v.isInputVariable())) {
                if(!isExternalVariable(v.getName()))
                    this.externalVariables.add(v);
            }
        }
    }

    public String getExternalVariableType(Variable extVar){
        if(extVar.getVariableType() != null){
            return extVar.getVariableType().getTypeNameTargetLanguage();
        }
        return "double";
    }

    public List<Variable> getExternalVariables(){
        return externalVariables;
    }

    public String getIpoptSolverFunctionCallParameters(){
        //original result = String.format("%s::solveOptimizationProblemIpOpt(%s, %s%s);\n", vm.getCallSolverName(), /*vm.getOptimizationVariables().get(0).getName()*/ "" , objVar, knownVariables);
        String result = "";
        //Pointer to optimization variables (return values)
        for (MathValueSymbol opt : getOptimizationVariables()){
            String varName = opt.getName();
            result += "&"+varName+", ";
        }
        //Objective variable (return value)
        result += "&"+getObjectiveVariableName()+", ";

        //External Variables, treated as constants regarding optimization
        for(Variable var : getExternalVariables()){
            String varName = var.getName();
            result += varName+", ";
        }

        if (result.length() >= 2) {
            result = result.substring(0, result.length() - 2);
        }
        return result;
    }

    public String getIpoptSolverFunctionParameters(){
        String result = "";
        //Pointer to optimization variables (return values)
        for (MathValueSymbol opt : getOptimizationVariables()){
            String varType = getVariableType(opt);
            String varName = opt.getName();
            result += varType+ " *"+varName+", ";
        }
        //Objective variable (return value)
        result += "double *"+getObjectiveVariableName()+", ";

        //External Variables, treated as constants regarding optimization
        for(Variable var : getExternalVariables()){
            String varType = var.getVariableType().getTypeNameTargetLanguage();
            String varName = var.getName();
            result += varType+ " "+varName+", ";
        }

        if (result.length() >= 2) {
            result = result.substring(0, result.length() - 2);
        }
        return result;
    }

    public String getVariableType(MathValueSymbol symbol){
        String result = "";
        String varTypeStr = symbol.getType().getType().getName();
        if (varTypeStr.contentEquals("Q"))
            result = "double";
        if(symbol.getType().getDimensions().size() > 0)
            result = "mat";
        if(hasStepSize())
            result = "colvec";
        return result;
    }

    public int getVariableDimensionM(MathValueSymbol symbol){
        List<MathExpressionSymbol> mesList = symbol.getType().getDimensions();
        if(mesList.size() >= 1){
            MathExpressionSymbol mes = mesList.get(0);
            MathNumberExpressionSymbol mnes = (MathNumberExpressionSymbol) mes;
            return mnes.getValue().getRealNumber().intValue();
        }
        return 1;
    }

    public int getVariableDimensionN(MathValueSymbol symbol){
        List<MathExpressionSymbol> mesList = symbol.getType().getDimensions();
        if(mesList.size() >= 2){
            MathExpressionSymbol mes = mesList.get(1);
            MathNumberExpressionSymbol mnes = (MathNumberExpressionSymbol) mes;
            return mnes.getValue().getRealNumber().intValue();
        }
        return 1;
    }

    public boolean isVarScalar(MathValueSymbol var){
        return (var.getType().getDimensions().isEmpty());
    }

    public int getOptVarDimension(){
        int result = 0;
        for(MathValueSymbol optVar : getOptimizationVariables()) {
            result += getVariableDimensionM(optVar) * getVariableDimensionN(optVar);

        }
        return result;
    }

    public int getIndVarDimension(){
        int result = 0;
        for(MathValueSymbol indVar : getIndependentVariables()) {
            result += getVariableDimensionM(indVar) * getVariableDimensionN(indVar);
        }
        return result;
    }

    public String getIpoptVarOffset(MathValueSymbol symbol){
        int var = -1;
        int pre_offset = 0;
        if(hasStepSize()) { //MPC
            if (getIndependentVariables().indexOf(symbol) != -1) {
                var = getIndependentVariables().indexOf(symbol);
                pre_offset = getNumberOptimizationVariables() * getStepSizeCount();
            }
            if (getOptimizationVariables().indexOf(symbol) != -1) {
                var = getOptimizationVariables().indexOf(symbol);
            }
            return pre_offset + " + " + getStepSizeCount() * var; //+ " * " + var;
        }else{
            int offset = 0;
            if (getOptimizationVariables().indexOf(symbol) != -1) {
                offset = 0;
                var = getOptimizationVariables().indexOf(symbol);
                for(int i = 0; i < getOptimizationVariables().indexOf(symbol);i++){
                    MathValueSymbol optVar = getOptimizationVariables().get(i);
                    offset += getVariableDimensionM(optVar) * getVariableDimensionN(optVar);
                }
            }
            if (getIndependentVariables().indexOf(symbol) != -1) {
                offset = 0;
                var = getOptimizationVariables().indexOf(symbol);
                for(MathValueSymbol optVar : getOptimizationVariables()){
                    pre_offset += getVariableDimensionM(optVar) * getVariableDimensionN(optVar);
                }
                for(int i = 0; i < getIndependentVariables().indexOf(symbol);i++){
                    MathValueSymbol indVar = getIndependentVariables().get(i);
                    offset += getVariableDimensionM(indVar) * getVariableDimensionN(indVar);
                }
            }
            return pre_offset + " + " + offset;
        }
    }

    public String getIpoptVarRef(MathValueSymbol symbol){return "_V_"+symbol.getName();}

    public String getIpoptVarRef(String symbolName){return "_V_"+symbolName;}

    public String getIpoptConstraintRef(int nr){return "_V_CONSTR"+nr;}

    public String getIpoptConstraintOffset(int nr){
        if(hasStepSize()) {
            int pre_offset = getNumberVariables() * getStepSizeCount();
            return pre_offset + " + " + getStepSizeCount() * nr;
        }else{
            int pre_offset = 0;
            for(MathValueSymbol optVar : getOptimizationVariables()){
                pre_offset += getVariableDimensionM(optVar) * getVariableDimensionN(optVar);
            }
            for(MathValueSymbol indVar : getIndependentVariables()){
                pre_offset += getVariableDimensionM(indVar) * getVariableDimensionN(indVar);
            }
            return pre_offset + " + " + nr;
        }
    }

    public String replaceVariablesWithIpoptVectorEntry(String code){
        String result = "";
        for (MathValueSymbol var : getOptimizationVariables()){
            code.replace(var.getName(), getIpoptVarRef(var));
        }
        for (MathValueSymbol var : getIndependentVariables()){
            code.replace(var.getName(), getIpoptVarRef( var));
        }
        return result;
    }

    public boolean hasStepSize(){
        if(getStepSize()!= null)
            return true;
        else
            return false;
    }

    public MathMatrixVectorExpressionSymbol getStepSizeVectorSymbol(){
        MathMatrixVectorExpressionSymbol result = null;
        MathExpressionSymbol mathExpression = getStepSize();
        if(mathExpression != null) {
            if (mathExpression.isAssignmentExpression()) {
                MathAssignmentExpressionSymbol assignExpression = (MathAssignmentExpressionSymbol) mathExpression;
                MathExpressionSymbol assignChildExpression = assignExpression.getExpressionSymbol();

                result = (MathMatrixVectorExpressionSymbol) assignChildExpression;
            }
        }
        return result;
    }

    public String getStepSizeName(){
        String result = "";
        MathExpressionSymbol mathExpression = getStepSize();
        if(mathExpression != null) {
            if (mathExpression.isAssignmentExpression()) {
                MathAssignmentExpressionSymbol assignExpression = (MathAssignmentExpressionSymbol) mathExpression;

                result = assignExpression.getNameOfMathValue();
            }
        }
        return result;
    }


    public int getStepSizeMin(){
        int result = 0;
        if(hasStepSize()) {
            MathMatrixVectorExpressionSymbol vectorExpressionSymbol = getStepSizeVectorSymbol();
            MathNumberExpressionSymbol startExpression = (MathNumberExpressionSymbol) vectorExpressionSymbol.getStart();
            result = startExpression.getValue().getRealNumber().intValue();
        }
        return result;
    }

    public int getStepSizeMax(){
        int result = 0;
        if(hasStepSize()) {
            MathMatrixVectorExpressionSymbol vectorExpressionSymbol = getStepSizeVectorSymbol();
            MathNumberExpressionSymbol endExpression = (MathNumberExpressionSymbol) vectorExpressionSymbol.getEnd();
            result =  endExpression.getValue().getRealNumber().intValue();
        }
        return result;
    }

    public int getStepSizeCount(){
        if(hasStepSize()) {
            return (getStepSizeMax() - getStepSizeMin() + 1);
        }else {
            return 1;
        }
    }

    public String getVariableInitialization(MathValueSymbol variable){
        String value = "0";
        if(variable.isValueExpression()){
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
        //getSimplifiedConditions rearranges all constraints, such that <const> <= <activeExpression>.
        //all upper bounds are automatically converted to lowerbounds.
        return "1E19";
    }

    public String getConstraintLowerBound(MathOptimizationConditionSymbol constraint){
        if(constraint.getLeft() != null) {
            if(containsOptScopedVariable(constraint.getLeft())){
                return "-1E19"; // This constraint is already covered by the variable constraints of the independent Variable
            } else {
                return constraint.getLeft().getTextualRepresentation();
            }
        }
        else if(constraint.getLowerBound().isPresent())
            return constraint.getLowerBound().get().getTextualRepresentation();
        else
            return "-1E19";
    }

    public String getObjectiveFunctionWithIpoptVectorEntries() {
        MathExpressionSymbol symbol = getObjectiveFunction();
        return getIpoptTextualRepresentation(symbol, "vars", 0);
    }

    public String getRawObjectiveFunction() {

        MathExpressionSymbol symbol = getObjectiveFunction();
        return ExecuteMethodGenerator.generateExecuteCode(symbol, new ArrayList<>());

    }


    public String listClassesInScope(){
        String result = "";
        if(!getIndependentVariables().isEmpty()) {
            for (Collection<Symbol> x : getIndependentVariables().get(0).getEnclosingScope().getLocalSymbols().values()) {
                result += "," + x.getClass().toString();
                for (Symbol s : x) {
                    result += ", " + s.getClass().toString();
                }
            }
        }
        return result;
    }

    public String transformEMAMValueToVectorNotation(MathNameExpressionSymbol symbol, String VectorName, Integer indexOffset){
        String result = "";
        String MatrixName = symbol.getNameToAccess();

        result = VectorName + "[" + getIpoptVarRef(MatrixName)+"]";

        return result;
    }

    public String transformEMAMMatrixAccessToVectorNotation(MathMatrixNameExpressionSymbol symbol, String VectorName, Integer indexOffset){
        String result = "";
        String MatrixName = symbol.getNameToAccess();
        String MatrixIndex = symbol.getMathMatrixAccessOperatorSymbol().getTextualRepresentation();

        result = VectorName + "[" + indexOffset +" + " +getIpoptVarRef(MatrixName)+" + "+ MatrixIndex + "]";

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
        if(getObjectiveVariableName().equals(varName))
            return true;

        return false;
    }

    public boolean containsOptScopedVariable(MathExpressionSymbol symbol){
        MathValueVisitor valueVisitor = new MathValueVisitor();
        MathAccessVisitor accessVisitor = new MathAccessVisitor();

        valueVisitor.handle(symbol);
        accessVisitor.handle(symbol);

        for (MathValueSymbol value : valueVisitor.getMathValueSymbols()){
            if(isOptScopedVariable(value.getName()))
                return true;
        }

        for (MathNameExpressionSymbol value : accessVisitor.getMathNameExpressionSymbols()){
            if(isOptScopedVariable(value.getNameToAccess()))
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

                //Exclude if MPC equation
                if(hasStepSize() && constraint.getOperator()=="=="){
                    modifiedConstraints.add(constraint);
                    continue;
                }

                //General form:     (lower <= expression <= upper)
                // We can expect expression to contain variables

                boolean lowerBoundContainsActiveVariables = false;
                if(constraint.getLowerBound().isPresent())
                    lowerBoundContainsActiveVariables = containsOptScopedVariable(constraint.getLowerBound().get());

                boolean upperBoundContainsActiveVariables = false;
                if(constraint.getUpperBound().isPresent())
                    upperBoundContainsActiveVariables = containsOptScopedVariable(constraint.getUpperBound().get());

                boolean expressionContainsActiveVariables = containsOptScopedVariable(constraint.getBoundedExpression());

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
                    Log.info("Constraint <"+constraint.getTextualRepresentation()+"> changed to <"+mocs.getTextualRepresentation()+".","MathOptConstraints");
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
                    Log.info("Constraint <"+constraint.getTextualRepresentation()+"> changed to <"+mocs.getTextualRepresentation()+".","MathOptConstraints");
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
                    Log.info("Constraint <"+constraint.getTextualRepresentation()+"> changed to <"+mocs.getTextualRepresentation()+".","MathOptConstraints");
                    handledConstraint = true;
                }
                if (!upperBoundContainsActiveVariables && !lowerBoundContainsActiveVariables && expressionContainsActiveVariables) {

                    if(constraint.getOperator()==">="){
                        //Swap left and right.
                        MathArithmeticExpressionSymbol maes = new MathArithmeticExpressionSymbol();
                        maes.setOperator("-");
                        maes.setLeftExpression(constraint.getBoundedExpression());
                        maes.setRightExpression(constraint.getLowerBound().get());
                        MathNumberExpressionSymbol mnes = new MathNumberExpressionSymbol();
                        mnes.setValue(new JSValue(Rational.ZERO));

                        MathOptimizationConditionSymbol mocs = new MathOptimizationConditionSymbol(mnes, "<=", maes);
                        modifiedConstraints.add(mocs);
                    }
                    if(constraint.getOperator()=="=="){
                        // 0 <= <constr> - <lower>
                        MathArithmeticExpressionSymbol maes = new MathArithmeticExpressionSymbol();
                        maes.setOperator("-");
                        maes.setLeftExpression(constraint.getBoundedExpression());
                        maes.setRightExpression(constraint.getLowerBound().get());
                        MathNumberExpressionSymbol mnes = new MathNumberExpressionSymbol();
                        mnes.setValue(new JSValue(Rational.ZERO));

                        MathOptimizationConditionSymbol mocs = new MathOptimizationConditionSymbol(mnes, "<=", maes);
                        modifiedConstraints.add(mocs);

                        // 0 <= <upper> - <constr>
                        maes = new MathArithmeticExpressionSymbol();
                        maes.setOperator("-");
                        maes.setLeftExpression(constraint.getUpperBound().get());
                        maes.setRightExpression(constraint.getBoundedExpression());
                        mnes = new MathNumberExpressionSymbol();
                        mnes.setValue(new JSValue(Rational.ZERO));

                        mocs = new MathOptimizationConditionSymbol(mnes, "<=", maes);
                        modifiedConstraints.add(mocs);
                    }
                    if(constraint.getOperator()=="<="){
                        //Let it through unchanged.
                        modifiedConstraints.add(constraint);

                    }
                    handledConstraint = true;
                }
                if(!handledConstraint){
                    Log.error("Constraint <"+constraint.getTextualRepresentation()+"> contains no active variables and is ignored.");
                }
            }
            return modifiedConstraints;
    }

    public int getSimplifiedConstraintFunctionCount(){
        return getSimplifiedConstraintFunctions().size();
    }

    private boolean replacementListContainsKey(List<String[]> list, String key){
        for (String[] element : list){
            if(element[0].equals(key))
                return true;
        }
        return false;
    }

    public String getIpoptTextualRepresentation(MathExpressionSymbol symbol, String vectorname, int offset){
        if(symbol==null)
            return "";

        MathAccessVisitor mav = new MathAccessVisitor();
        String textRep = symbol.getTextualRepresentation();
        mav.handle(symbol);

        List<String[]> replacementList = new ArrayList<>();
        for (MathMatrixNameExpressionSymbol matAccess : mav.getMathMatrixNameExpressionSymbols()){
            if(isOptScopedVariable(matAccess.getNameToAccess())) {
                String replacement = transformEMAMMatrixAccessToVectorNotation(matAccess, vectorname, offset);
                String search = matAccess.getTextualRepresentation();
                String replacementPair[] = {search, replacement};
                if(!replacementListContainsKey(replacementList, search))
                    replacementList.add(replacementPair);
            }
        }
        for (MathNameExpressionSymbol matAccess : mav.getMathNameExpressionSymbols()){
            if(isOptScopedVariable(matAccess.getNameToAccess())) {
                String replacement = transformEMAMValueToVectorNotation(matAccess, vectorname, offset);
                String search = matAccess.getTextualRepresentation();
                String replacementPair[] = {search, replacement};

                if(!replacementListContainsKey(replacementList, search))
                    replacementList.add(replacementPair);
            }
        }
        for(String replacementPair[] : replacementList) {
            textRep = textRep.replace(replacementPair[0],replacementPair[1]);
        }
        return textRep;
    }

    public String getConstraintForFG_Eval(MathExpressionSymbol constraint, int nr){
        String debug = "";
        String result="";

        if(constraint instanceof MathOptimizationConditionSymbol) {

            MathOptimizationConditionSymbol optCondition = (MathOptimizationConditionSymbol) constraint;

            if(optCondition.getOperator().equals("==") && hasStepSize()) {
                if(optCondition.getLeft() instanceof MathMatrixNameExpressionSymbol) {
                    //MPC
                    MathMatrixNameExpressionSymbol left = (MathMatrixNameExpressionSymbol) optCondition.getLeft();
                    String leftSide = transformEMAMMatrixAccessToVectorNotation(left,"fg",1);
                    String rightSide = getIpoptTextualRepresentation(optCondition.getRight(),"vars",0);

                    result = leftSide+" = "+rightSide;
                }else{

                }
            } else {
                String leftside = "fg[ 1 +" + getIpoptConstraintRef(nr) + " ] ";
                //String rightside = getIpoptTextualRepresentation(((MathOptimizationConditionSymbol) constraint).getRight(), "vars", 0);
                String rightside = ExecuteMethodGenerator.generateExecuteCode(((MathOptimizationConditionSymbol) constraint).getRight(), new ArrayList<>());
                result = leftside + " = " + rightside;
            }

        }
        return result;
    }


    //ToDo: Write generic Visitor/Inspector and move it to MontiMath?
    private class MathAccessVisitor implements MathOptExpressionSymbolVisitor{

        private List<MathMatrixNameExpressionSymbol> mathMatrixNameExpressionSymbols = new ArrayList<>();

        private List<MathNameExpressionSymbol> mathNameExpressionSymbols = new ArrayList<>();
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

        @Override
        public void visit(MathNameExpressionSymbol node) {
            mathNameExpressionSymbols.add(node);
        }

        public List<MathMatrixNameExpressionSymbol> getMathMatrixNameExpressionSymbols() {
            return mathMatrixNameExpressionSymbols;
        }

        public List<MathNameExpressionSymbol> getMathNameExpressionSymbols() {
            return mathNameExpressionSymbols;
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
