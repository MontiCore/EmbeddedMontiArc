///* (c) https://github.com/MontiCore/monticore */
//package de.monticore.lang.monticar.semantics.solver.linearsolver;
//
//import de.monticore.lang.math._symboltable.expression.*;
//import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
//import de.se_rwth.commons.logging.Log;
//
//import java.util.ArrayList;
//import java.util.HashSet;
//import java.util.Map;
//import java.util.Set;
//
//public class RowToCoefficients extends MathExpressionSymbolVisitor {
//
//    private ArrayList<String> row;
//    private String solution;
//    private Set<String> constants = new HashSet<>();
//
//    private Set<String> variables;
//    private Map<String, Integer> mappingToIndex;
//
//    private boolean leftOfAssignment;
//    private boolean negative;
//    private boolean asCoefficient;
//
//
//    public ArrayList<String> getRow() {
//        return row;
//    }
//
//    public String getSolution() {
//        return solution;
//    }
//
//    RowToCoefficients(Set<String> variables, Map<String, Integer> mappingToIndex) {
//        this.variables = variables;
//        this.mappingToIndex = mappingToIndex;
//
//        row = new ArrayList<>(variables.size() + 1);
//        for (int i = 0; i < variables.size(); i++) {
//            row.add("0");
//        }
//
//        solution = "0";
//    }
//
//    private String getSign() {
//        return negative ^ leftOfAssignment ^ asCoefficient ? "-" : "";
//    }
//
//    private void addToIndex(int i, String val) {
//        String toAdd = row.get(i);
//        val = getSign() + "(" + val + ")";
//        if (toAdd.equals("0"))
//            toAdd = val;
//        else if (val.startsWith("-"))
//            toAdd = toAdd + val;
//        else
//            toAdd = toAdd + " + " + val;
//        ((ArrayList) row).set(i, toAdd);
//    }
//
//    private void addToSolution(String val) {
//        asCoefficient = true;
//        val = getSign() + "(" + val + ")";
//        asCoefficient = false;
//        if (solution.equals("0"))
//            solution = val;
//        else if(val.startsWith("-"))
//            solution = solution + val;
//        else
//            solution = solution + " + " + val;
//    }
//
//    @Override
//    public void traverse(MathAssignmentExpressionSymbol expr) {
//        leftOfAssignment = true;
//        if (variables.contains(expr.getNameOfMathValue()))
//            addToIndex(mappingToIndex.get(expr.getNameOfMathValue()), "1");
//        else
//            addToSolution(expr.getNameOfMathValue());
//        leftOfAssignment = false;
//        handle(expr.getExpressionSymbol());
//    }
//
//    @Override
//    public void traverse(MathArithmeticExpressionSymbol expr) {
//        if (expr.getMathOperator().equals("-")) {
//            handle(expr.getLeftExpression());
//            negative = !negative;
//            handle(expr.getRightExpression());
//            negative = !negative;
//        } else if (expr.getMathOperator().equals("+")) {
//            handle(expr.getLeftExpression());
//            handle(expr.getRightExpression());
//        } else if (expr.getMathOperator().equals("*")) {
//            if (expr.getLeftExpression() instanceof MathNameExpressionSymbol &&
//                    variables.contains(((MathNameExpressionSymbol) expr.getLeftExpression()).getNameToResolveValue())) {
//                String var = ((MathNameExpressionSymbol) expr.getLeftExpression()).getNameToResolveValue();
//                String coefficient = expr.getRightExpression().getTextualRepresentation();
//
//                addToIndex(mappingToIndex.get(var), coefficient);
//            } else if (expr.getRightExpression() instanceof MathNameExpressionSymbol &&
//                    variables.contains(((MathNameExpressionSymbol) expr.getRightExpression()).getNameToResolveValue()))  {
//                String var =  ((MathNameExpressionSymbol) expr.getRightExpression()).getNameToResolveValue();
//                String coefficient = expr.getLeftExpression().getTextualRepresentation();
//
//                addToIndex(mappingToIndex.get(var), coefficient);
//            } else {
//                String coefficient = expr.getTextualRepresentation();
//
//                addToSolution(coefficient);
//            }
//        } else if (expr.getMathOperator().equals("/")) {
//            if (expr.getLeftExpression() instanceof MathNameExpressionSymbol &&
//                    variables.contains(((MathNameExpressionSymbol) expr.getLeftExpression()).getNameToResolveValue())) {
//                String var = ((MathNameExpressionSymbol) expr.getLeftExpression()).getNameToResolveValue();
//                String coefficient = "1/(" + expr.getRightExpression().getTextualRepresentation() + ")";
//
//                addToIndex(mappingToIndex.get(var), coefficient);
//            } else if (expr.getRightExpression() instanceof MathNameExpressionSymbol &&
//                    variables.contains(((MathNameExpressionSymbol) expr.getRightExpression()).getNameToResolveValue()))  {
//                // TODO
//                Log.error("0x012305 Non Linear equation System");
//            } else {
//                String coefficient = expr.getTextualRepresentation();
//
//                addToSolution(coefficient);
//            }
//        } else {
//            Log.error("0x458654 Not supported");
//        }
//    }
//
//    @Override
//    public void visit(MathNumberExpressionSymbol expr) {
//        String val = "" + expr.getValue().getRealNumber().toString();
//        addToSolution(val);
//    }
//
//    @Override
//    public void visit(MathNameExpressionSymbol expr) {
//        if (variables.contains(expr.getNameToResolveValue())) {
//            addToIndex(mappingToIndex.get(expr.getNameToResolveValue()), "1");
//        } else {
//            addToSolution(expr.getNameToResolveValue());
//            constants.add(expr.getNameToResolveValue());
//        }
//    }
//
//    public Set<String> getConstants() {
//        return constants;
//    }
//}
