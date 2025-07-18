/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class MathForLoopExpressionSymbol extends MathExpressionSymbol {

    protected MathForLoopHeadSymbol forLoopHead;

    protected List<MathExpressionSymbol> forLoopBody = new ArrayList<>();

    public MathForLoopExpressionSymbol() {
        super();
    }

    public MathForLoopHeadSymbol getForLoopHead() {
        return forLoopHead;
    }

    public void setForLoopHead(MathForLoopHeadSymbol forLoopHead) {
        this.forLoopHead = forLoopHead;
    }

    public List<MathExpressionSymbol> getForLoopBody() {
        return forLoopBody;
    }

    public void addForLoopBody(MathExpressionSymbol forLoopBody) {
        this.forLoopBody.add(forLoopBody);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        result += "for(" + forLoopHead.getNameLoopVariable() + "=" + forLoopHead.getMathExpression().getTextualRepresentation() + ")";
        result += "{\n";

        for (MathExpressionSymbol bodyExpr : forLoopBody) {
            result += bodyExpr.getTextualRepresentation() + ";\n";
        }
        result += "}\n";

        return result;
    }

    @Override
    public boolean isForLoopExpression() {
        return true;
    }
}
