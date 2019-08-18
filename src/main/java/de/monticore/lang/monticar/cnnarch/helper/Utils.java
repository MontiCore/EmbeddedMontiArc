/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnnarch.helper;

import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.monticar.cnnarch._symboltable.TupleExpressionSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.*;
import java.util.function.Function;

public class Utils {


    public static List<MathExpressionSymbol> createSubExpressionList(MathExpressionSymbol expression){
        List<MathExpressionSymbol> list = new LinkedList<>();
        list.add(expression);

        //switch expression.class
        if (expression instanceof MathParenthesisExpressionSymbol){
            MathParenthesisExpressionSymbol exp = (MathParenthesisExpressionSymbol) expression;
            list.addAll(createSubExpressionList(exp.getMathExpressionSymbol()));
        }
        else if (expression instanceof MathCompareExpressionSymbol){
            MathCompareExpressionSymbol exp = (MathCompareExpressionSymbol) expression;
            list.addAll(createSubExpressionList(exp.getLeftExpression()));
            list.addAll(createSubExpressionList(exp.getRightExpression()));
        }
        else if (expression instanceof MathArithmeticExpressionSymbol){
            MathArithmeticExpressionSymbol exp = (MathArithmeticExpressionSymbol) expression;
            list.addAll(createSubExpressionList(exp.getLeftExpression()));
            list.addAll(createSubExpressionList(exp.getRightExpression()));
        }
        else if (expression instanceof MathPreOperatorExpressionSymbol){
            MathPreOperatorExpressionSymbol exp = (MathPreOperatorExpressionSymbol) expression;
            list.addAll(createSubExpressionList(exp.getMathExpressionSymbol()));
        }
        else if (expression instanceof TupleExpressionSymbol){
            TupleExpressionSymbol tuple = (TupleExpressionSymbol) expression;
            for (MathExpressionSymbol exp : tuple.getExpressions()){
                list.addAll(createSubExpressionList(exp));
            }
        }
        else if (expression instanceof MathValueExpressionSymbol){
            //do nothing
        }
        else {
            throw new IllegalArgumentException("Unknown expression type: " + expression.getClass().getSimpleName());
        }

        return list;
    }

    public static String replace(String expression, Map<String, String> replacementMap){
        String resolvedString = expression;
        for (String name : replacementMap.keySet()){
            resolvedString = resolvedString.replaceAll(name, replacementMap.get(name));
        }
        return resolvedString;
    }

    public static <T> String createTupleTextualRepresentation(List<T> list, Function<T,String> stringFunction){
        StringBuilder builder = new StringBuilder();
        builder.append("(");
        for (int i = 0; i< list.size(); i++){
            builder.append(stringFunction.apply(list.get(i)));
            if (i != list.size()-1 || i==0){
                builder.append(",");
            }
        }
        builder.append(")");
        return builder.toString();
    }

    public static MathExpressionSymbol copy(MathExpressionSymbol expression){
        MathExpressionSymbol copy;

        //switch expression.class
        if (expression instanceof MathParenthesisExpressionSymbol){
            MathParenthesisExpressionSymbol exp = (MathParenthesisExpressionSymbol) expression;
            copy = new MathParenthesisExpressionSymbol(copy(exp.getMathExpressionSymbol()));
        }
        else if (expression instanceof MathCompareExpressionSymbol){
            MathCompareExpressionSymbol exp = (MathCompareExpressionSymbol) expression;
            MathCompareExpressionSymbol castedCopy = new MathCompareExpressionSymbol();
            castedCopy.setCompareOperator(exp.getCompareOperator());
            castedCopy.setLeftExpression(copy(exp.getLeftExpression()));
            castedCopy.setRightExpression(copy(exp.getRightExpression()));
            copy = castedCopy;
        }
        else if (expression instanceof MathArithmeticExpressionSymbol){
            MathArithmeticExpressionSymbol exp = (MathArithmeticExpressionSymbol) expression;
            MathArithmeticExpressionSymbol castedCopy = new MathArithmeticExpressionSymbol();
            castedCopy.setMathOperator(exp.getMathOperator());
            castedCopy.setLeftExpression(copy(exp.getLeftExpression()));
            castedCopy.setRightExpression(copy(exp.getRightExpression()));
            copy = castedCopy;
        }
        else if (expression instanceof MathPreOperatorExpressionSymbol){
            MathPreOperatorExpressionSymbol exp = (MathPreOperatorExpressionSymbol) expression;
            MathPreOperatorExpressionSymbol castedCopy = new MathPreOperatorExpressionSymbol();
            castedCopy.setOperator(exp.getOperator());
            castedCopy.setMathExpressionSymbol(copy(exp.getMathExpressionSymbol()));
            copy = castedCopy;
        }
        else if (expression instanceof TupleExpressionSymbol){
            TupleExpressionSymbol exp = (TupleExpressionSymbol) expression;
            TupleExpressionSymbol castedCopy = new TupleExpressionSymbol();
            List<MathExpressionSymbol> elementCopies = new ArrayList<>();
            for (MathExpressionSymbol element : exp.getExpressions()){
                elementCopies.add(copy(element));
            }
            castedCopy.setExpressions(elementCopies);
            copy = castedCopy;
        }
        else if (expression instanceof MathNameExpressionSymbol){
            MathNameExpressionSymbol exp = (MathNameExpressionSymbol) expression;
            copy = new MathNameExpressionSymbol(exp.getNameToResolveValue());
        }
        else if (expression instanceof MathNumberExpressionSymbol){
            MathNumberExpressionSymbol exp = (MathNumberExpressionSymbol) expression;
            copy = new MathNumberExpressionSymbol(exp.getValue().getRealNumber());
        }
        else if (expression instanceof MathBooleanExpressionSymbol){
            MathBooleanExpressionSymbol castedCopy = (MathBooleanExpressionSymbol) expression.getRealMathExpressionSymbol();
            copy = castedCopy;
        }
        else {
            throw new IllegalArgumentException("Unknown expression type: " + expression.getClass().getSimpleName());
        }

        copy.setID(expression.getExpressionID());
        if (expression.getAstNode().isPresent()){
            copy.setAstNode(expression.getAstNode().get());
        }
        return copy;
    }

    public static boolean equals(ASTElementType firstType, ASTElementType secondType){
        if (firstType.isBoolean() ^ secondType.isBoolean()
                || firstType.isNaturalNumber() ^ secondType.isNaturalNumber()
                || firstType.isRational() ^ secondType.isRational()
                || firstType.isWholeNumber() ^ secondType.isWholeNumber()
                || firstType.isComplex() ^ secondType.isComplex()){
            return false;
        }
        if (firstType.isPresentRange()){
            if (!secondType.isPresentRange()){
                return false;
            }
        }
        else {
            return !secondType.isPresentRange();
        }

        return equals(firstType.getRange(), secondType.getRange());
    }

    public static boolean equals(ASTRange firstRange, ASTRange secondRange){
        if (firstRange.hasNoLowerLimit() ^ secondRange.hasNoLowerLimit()
                || firstRange.hasNoUpperLimit() ^ secondRange.hasNoUpperLimit()){
            return false;
        }
        if (!firstRange.hasNoLowerLimit() && !firstRange.getStartValue().equals(secondRange.getStartValue())){
            return false;
        }
        if (!firstRange.hasNoUpperLimit() && !firstRange.getEndValue().equals(secondRange.getEndValue())){
            return false;
        }
        if (firstRange.isPresentStep() ^ secondRange.isPresentStep()){
            return false;
        }
        if (firstRange.isPresentStep() && !firstRange.getStepValue().equals(secondRange.getStepValue())){
            return false;
        }

        return true;
    }

    // Checks if firstType contains secondType
    public static boolean contains(ASTElementType firstType, ASTElementType secondType){
        if (firstType.isBoolean()) {
            if (!secondType.isBoolean()) {
                return false;
            }
        }

        if (firstType.isNaturalNumber()) {
            if (secondType.isWholeNumber() || secondType.isRational() || secondType.isComplex()) {
                return false;
            }
        }

        if (firstType.isWholeNumber()) {
            if (secondType.isRational() || secondType.isComplex()) {
                return false;
            }
        }

        if (firstType.isRational()) {
            if (secondType.isComplex()) {
                return false;
            }
        }

        if (firstType.isPresentRange()) {
            if (!secondType.isPresentRange()) {
                return false;
            }
        }
        else {
            return true;
        }

        return contains(firstType.getRange(), secondType.getRange());
    }

    public static boolean contains(ASTRange firstRange, ASTRange secondRange) {
        if (!firstRange.hasNoLowerLimit() && secondRange.hasNoLowerLimit()) {
            return false;
        }

        if (!firstRange.hasNoUpperLimit() && secondRange.hasNoUpperLimit()) {
            return false;
        }

        if (!firstRange.hasNoLowerLimit() && firstRange.getStartValue().compareTo(secondRange.getStartValue()) > 0) {
            return false;
        }

        if (!firstRange.hasNoUpperLimit() && firstRange.getEndValue().compareTo(secondRange.getEndValue()) < 0){
            return false;
        }

        if (firstRange.isPresentStep()){
            if (!secondRange.isPresentStep()) {
                return false;
            }
        }
        else {
            return true;
        }

        if (!firstRange.getStepValue().equals(secondRange.getStepValue())){
            return false;
        }

        // TODO: Check for different step sizes; too complex and not necessary for current use

        return true;
    }

    public static void recursiveSetResolvingFilters(Scope scope, Collection<ResolvingFilter<? extends Symbol>> resolvingFilters){
        scope.getAsMutableScope().setResolvingFilters(resolvingFilters);
        for (Scope subScope : scope.getSubScopes()){
            recursiveSetResolvingFilters(subScope, resolvingFilters);
        }
    }
}
