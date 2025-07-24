/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.Optional;

/**
 */
public class MathMatrixVectorExpressionSymbol extends MathMatrixExpressionSymbol {
    protected MathExpressionSymbol start;
    protected Optional<MathExpressionSymbol> step = Optional.empty();
    protected MathExpressionSymbol end;

    public MathMatrixVectorExpressionSymbol() {
        super();
    }


    public MathExpressionSymbol getStart() {
        return start;
    }

    public void setStart(MathExpressionSymbol start) {
        this.start = start;
    }

    public Optional<MathExpressionSymbol> getStep() {
        return step;
    }

    public void setStep(MathExpressionSymbol step) {
        this.step = Optional.of(step);
    }

    public MathExpressionSymbol getEnd() {
        return end;
    }

    public void setEnd(MathExpressionSymbol end) {
        this.end = end;
    }

    @Override
    public String getTextualRepresentation() {
        if (!step.isPresent()) {
            return start.getTextualRepresentation() + ":" + end.getTextualRepresentation();
        }
        return start.getTextualRepresentation() + ":" + step.get().getTextualRepresentation() + ":" + end.getTextualRepresentation();
    }

    @Override
    public boolean isMatrixVectorExpression() {
        return true;
    }
}
