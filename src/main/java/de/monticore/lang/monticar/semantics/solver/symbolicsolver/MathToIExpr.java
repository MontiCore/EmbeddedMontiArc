/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolicsolver;

import de.monticore.assignmentexpressions._ast.*;
import de.monticore.commonexpressions._ast.*;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.se_rwth.commons.logging.Log;
import org.matheclipse.core.expression.F;
import org.matheclipse.core.expression.Num;
import org.matheclipse.core.interfaces.IExpr;

public class MathToIExpr {

    public IExpr convert(ASTExpression expression) {
        if (!(expression instanceof ASTRegularAssignmentExpression))
            Log.error("0x something");
        Converter converter = new Converter();
        expression.accept(converter);
        return converter.result;
    }

    private class Converter implements EmbeddedMontiArcMathVisitor {

        IExpr result;
        boolean regularAssignmentSeen = false;

        private Converter() {

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
        public void visit(ASTAssignmentExpression expr){

        }

        @Override
        public void traverse(ASTRegularAssignmentExpression expr) {
            expr.getLeftExpression().accept(getRealThis());
            IExpr leftResult = result;
            expr.getRightExpression().accept(getRealThis());
            IExpr rightResult = result;

            result = F.Equal(leftResult, rightResult);
        }

        @Override
        public void visit(ASTMinusPrefixExpression expr){

        }

        @Override
        public void visit(ASTPlusPrefixExpression expr){

        }

        @Override
        public void visit(ASTDecPrefixExpression expr){

        }

        @Override
        public void visit(ASTDecSuffixExpression expr){

        }

        @Override
        public void visit(ASTIncPrefixExpression expr){

        }

        @Override
        public void visit(ASTIncSuffixExpression expr){

        }

        /*************************************************COMMON EXPRESSIONS****************************************************/

        @Override
        public void visit(ASTGreaterEqualExpression expr) {

        }

        @Override
        public void visit(ASTLessEqualExpression expr){

        }

        @Override
        public void visit(ASTGreaterThanExpression expr){

        }

        @Override
        public void visit(ASTLessThanExpression expr){

        }

        @Override
        public void traverse(ASTPlusExpression expr){
            expr.getLeftExpression().accept(getRealThis());
            IExpr leftResult = result;
            expr.getRightExpression().accept(getRealThis());
            IExpr rightResult = result;

            result = F.Plus(leftResult, rightResult);
        }

        @Override
        public void traverse(ASTMinusExpression expr){
            expr.getLeftExpression().accept(getRealThis());
            IExpr leftResult = result;
            expr.getRightExpression().accept(getRealThis());
            IExpr rightResult = result;

            result = F.Subtract(leftResult, rightResult);
        }

        @Override
        public void traverse(ASTMultExpression expr){
            expr.getLeftExpression().accept(getRealThis());
            IExpr leftResult = result;
            expr.getRightExpression().accept(getRealThis());
            IExpr rightResult = result;

            result = F.Times(leftResult, F.Negate(rightResult));
        }

        @Override
        public void traverse(ASTDivideExpression expr){
            expr.getLeftExpression().accept(getRealThis());
            IExpr leftResult = result;
            expr.getRightExpression().accept(getRealThis());
            IExpr rightResult = result;

            result = F.Times(leftResult, F.Power(rightResult,F.CN1));
        }

        @Override
        public void visit(ASTModuloExpression expr){

        }

        @Override
        public void visit(ASTEqualsExpression expr){

        }

        @Override
        public void visit(ASTNotEqualsExpression expr){

        }

        @Override
        public void visit(ASTCallExpression expr){

        }

        @Override
        public void visit(ASTLogicalNotExpression expr){

        }

        @Override
        public void visit(ASTBooleanAndOpExpression expr){

        }

        @Override
        public void visit(ASTBooleanOrOpExpression expr){

        }

        @Override
        public void visit(ASTBooleanNotExpression expr){

        }

        @Override
        public void visit(ASTBracketExpression expr){

        }

        @Override
        public void visit(ASTConditionalExpression expr){

        }

        @Override
        public void visit(ASTArguments expr){

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
        public void visit(ASTBinaryOrOpExpression expr){

        }

        @Override
        public void visit(ASTBinaryAndExpression expr){

        }

        @Override
        public void visit(ASTBinaryXorExpression expr){

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
            double val = expr.getNumberWithUnit().getNumber().get().doubleValue();
            result = Num.valueOf(val);
        }

        @Override
        public void visit(ASTNameExpression expr){
            result = F.symbol(expr.getName());
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
}
