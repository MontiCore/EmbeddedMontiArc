/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.linearsolver;

import de.monticore.assignmentexpressions._ast.*;
import de.monticore.commonexpressions._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.semantics.util.ExpressionPrettyPrinter;

import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

public class RowToCoefficients implements EmbeddedMontiArcMathVisitor {

    private ArrayList<String> row;
    String solution;

    private Set<String> variables;
    private Map<String, Integer> mappingToIndex;

    private boolean leftOfAssignment;
    private boolean negative;
    private boolean asCoefficient;


    public ArrayList<String> getRow() {
        return row;
    }

    public String getSolution() {
        return solution;
    }

    RowToCoefficients(Set<String> variables, Map<String, Integer> mappingToIndex) {
        this.variables = variables;
        this.mappingToIndex = mappingToIndex;

        row = new ArrayList<>(variables.size() + 1);
        for (int i = 0; i < variables.size(); i++) {
            row.add("0");
        }

        solution = "0";
    }

    private String getSign() {
        return negative ^ leftOfAssignment ^ asCoefficient ? "-" : "";
    }

    private void addToIndex(int i, String val) {
        String toAdd = row.get(i);
        val = getSign() + "(" + val + ")";
        if (toAdd.equals("0"))
            toAdd = val;
        else if (val.startsWith("-"))
            toAdd = toAdd + val;
        else
            toAdd = toAdd + " + " + val;
        ((ArrayList) row).set(i, toAdd);
    }

    private void addToSolution(String val) {
        asCoefficient = true;
        val = getSign() + "(" + val + ")";
        asCoefficient = false;
        if (solution.equals("0"))
            solution = val;
        else if(val.startsWith("-"))
            solution = solution + val;
        else
            solution = solution + " + " + val;
    }



    private EmbeddedMontiArcMathVisitor realThis = this;

    @Override
    public EmbeddedMontiArcMathVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EmbeddedMontiArcMathVisitor realThis) {
        this.realThis = realThis;
    }

    /*************************************************ASSIGNMENT EXPRESSIONS****************************************************/
    @Override
    public void visit(ASTAssignmentExpression expr) {

    }

    @Override
    public void traverse(ASTRegularAssignmentExpression expr) {
        leftOfAssignment = true;
        expr.getLeftExpression().accept(getRealThis());
        leftOfAssignment = false;
        expr.getRightExpression().accept(getRealThis());
    }

    @Override
    public void visit(ASTMinusPrefixExpression expr) {

    }

    @Override
    public void visit(ASTPlusPrefixExpression expr) {

    }

    @Override
    public void visit(ASTDecPrefixExpression expr) {

    }

    @Override
    public void visit(ASTDecSuffixExpression expr) {

    }

    @Override
    public void visit(ASTIncPrefixExpression expr) {

    }

    @Override
    public void visit(ASTIncSuffixExpression expr) {

    }

    /*************************************************COMMON EXPRESSIONS****************************************************/

    @Override
    public void visit(ASTGreaterEqualExpression expr) {

    }

    @Override
    public void visit(ASTLessEqualExpression expr) {

    }

    @Override
    public void visit(ASTGreaterThanExpression expr) {

    }

    @Override
    public void visit(ASTLessThanExpression expr) {

    }

    @Override
    public void visit(ASTPlusExpression expr) {

    }

    @Override
    public void traverse(ASTMinusExpression expr) {
        expr.getLeftExpression().accept(getRealThis());
        negative = !negative;
        expr.getRightExpression().accept(getRealThis());
        negative = !negative;
    }

    @Override
    public void traverse(ASTMultExpression expr) {
        if (expr.getLeftExpression() instanceof ASTNameExpression &&
                variables.contains(((ASTNameExpression) expr.getLeftExpression()).getName())) {
            String var = ((ASTNameExpression) expr.getLeftExpression()).getName();
            String coefficient = ExpressionPrettyPrinter.prettyPrint(expr.getRightExpression());

            addToIndex(mappingToIndex.get(var), coefficient);
        } else if (expr.getRightExpression() instanceof ASTNameExpression &&
                variables.contains(((ASTNameExpression) expr.getRightExpression()).getName())) {
            String var = ((ASTNameExpression) expr.getRightExpression()).getName();
            String coefficient = ExpressionPrettyPrinter.prettyPrint(expr.getLeftExpression());

            addToIndex(mappingToIndex.get(var), coefficient);
        } else {
            String coefficient = ExpressionPrettyPrinter.prettyPrint(expr);

            addToSolution(coefficient);
        }
    }

    @Override
    public void traverse(ASTDivideExpression expr) {
        if (expr.getLeftExpression() instanceof ASTNameExpression &&
                variables.contains(((ASTNameExpression) expr.getLeftExpression()).getName())) {
            String var = ((ASTNameExpression) expr.getLeftExpression()).getName();
            String coefficient = "1/(" + ExpressionPrettyPrinter.prettyPrint(expr.getRightExpression()) + ")";

            addToIndex(mappingToIndex.get(var), coefficient);
        } else if (expr.getRightExpression() instanceof ASTNameExpression &&
                variables.contains(((ASTNameExpression) expr.getRightExpression()).getName())) {
            // TODO
            return;
        } else {
            String coefficient = ExpressionPrettyPrinter.prettyPrint(expr);
            addToSolution(coefficient);
        }
    }

    @Override
    public void visit(ASTModuloExpression expr) {

    }

    @Override
    public void visit(ASTEqualsExpression expr) {

    }

    @Override
    public void visit(ASTNotEqualsExpression expr) {

    }

    @Override
    public void visit(ASTCallExpression expr) {

    }

    @Override
    public void visit(ASTLogicalNotExpression expr) {

    }

    @Override
    public void visit(ASTBooleanAndOpExpression expr) {

    }

    @Override
    public void visit(ASTBooleanOrOpExpression expr) {

    }

    @Override
    public void visit(ASTBooleanNotExpression expr) {

    }

    @Override
    public void visit(ASTBracketExpression expr) {

    }

    @Override
    public void visit(ASTConditionalExpression expr) {

    }

    @Override
    public void visit(ASTArguments expr) {

    }

    /*************************************************BIT EXPRESSIONS****************************************************/

//        @Override
//        public void visit(ASTLogicalRightShiftExpression expr) {
//
//        }
//
//        @Override
//        public void visit(ASTRightShiftExpression expr) {
//
//        }
//
//        @Override
//        public void visit(ASTLeftShiftExpression expr) {
//
//        }
    @Override
    public void visit(ASTBinaryOrOpExpression expr) {

    }

    @Override
    public void visit(ASTBinaryAndExpression expr) {

    }

    @Override
    public void visit(ASTBinaryXorExpression expr) {

    }

    /*************************************************SET EXPRESSIONS****************************************************/

//        @Override
//        public void visit(ASTIsInExpression expr){
//
//        }
//
//        @Override
//        public void visit(ASTSetInExpression expr){
//
//        }
//
//        @Override
//        public void visit(ASTUnionExpressionInfix expr){
//
//        }
//
//        @Override
//        public void visit(ASTIntersectionExpressionInfix expr){
//
//        }

    /*************************************************EXPRESSIONS BASIS****************************************************/

//        @Override
//        public void visit(ASTLiteralExpression expr){
//
//            expr.getLiteral().setEnclosingScope(getScope());
//        }
    @Override
    public void visit(ASTNumberExpression expr) {
        String val = "" + expr.getNumberWithUnit().getNumber().get().doubleValue();
        addToSolution(val);
    }

    @Override
    public void visit(ASTNameExpression expr) {
        if (variables.contains(expr.getName())) {
            addToIndex(mappingToIndex.get(expr.getName()), "1");
        } else {
            addToSolution(expr.getName());
        }
    }

//        @Override
//        public void visit(ASTMCQualifiedType type){
//            type.setEnclosingScope(getScope());
//        }
//
//        @Override
//        public void visit(ASTMCQualifiedName name) {
//            name.setEnclosingScope(getScope());
//        }
//
//        @Override
//        public void visit(ASTMCReturnType type){
//            type.setEnclosingScope(getScope());
//        }
}
