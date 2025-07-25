/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._ast.ASTMathMatrixNameExpression;
import de.monticore.lang.math._symboltable.expression.IMathNamedExpression;

import java.util.Optional;

/**
 */
public class MathMatrixNameExpressionSymbol extends MathMatrixExpressionSymbol implements IMathNamedExpression {

    protected Optional<ASTMathMatrixNameExpression> astMathMatrixNameExpression = Optional.empty();
    protected String nameToAccess;
    protected Optional<MathMatrixAccessOperatorSymbol> mathMatrixAccessOperatorSymbol = Optional.empty();

    public MathMatrixNameExpressionSymbol(String nameToAccess) {
        super();
        this.nameToAccess = nameToAccess;
    }

    public boolean isASTMathMatrixNamePresent() {
        return astMathMatrixNameExpression.isPresent();
    }

    public ASTMathMatrixNameExpression getAstMathMatrixNameExpression() {
        return astMathMatrixNameExpression.get();
    }

    public void setAstMathMatrixNameExpression(ASTMathMatrixNameExpression astMathMatrixNameExpression) {
        this.astMathMatrixNameExpression = Optional.of(astMathMatrixNameExpression);
    }

    public String getNameToAccess() {
        return nameToAccess;
    }

    public void setNameToAccess(String nameToAccess) {
        this.nameToAccess = nameToAccess;
    }

    public boolean hasEndOperator() {
        return astMathMatrixNameExpression.get().getMathMatrixAccessExpression().getMathMatrixAccessList().size() > 1;
    }

    public boolean hasMatrixAccessExpression() {
        return astMathMatrixNameExpression.isPresent() && (!astMathMatrixNameExpression.get().getMathMatrixAccessExpression().isEmptyMathMatrixAccesss());
    }

    public MathMatrixAccessOperatorSymbol getMathMatrixAccessOperatorSymbol() {
        return mathMatrixAccessOperatorSymbol.get();
    }

    public boolean isMathMatrixAccessOperatorSymbolPresent() {
        return mathMatrixAccessOperatorSymbol.isPresent();
    }

    public void setMathMatrixAccessOperatorSymbol(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        if (astMathMatrixNameExpression.isPresent()) {
            astMathMatrixNameExpression.get().getMathMatrixAccessExpression().setSymbol(mathMatrixAccessOperatorSymbol);
        }
        this.mathMatrixAccessOperatorSymbol = Optional.of(mathMatrixAccessOperatorSymbol);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        result += nameToAccess;
//        result += "(";
        result += getMathMatrixAccessOperatorSymbol().getTextualRepresentation();
//        result += ")";
        return result;
    }

    @Override
    public boolean isMatrixNameExpression() {
        return true;
    }


}
