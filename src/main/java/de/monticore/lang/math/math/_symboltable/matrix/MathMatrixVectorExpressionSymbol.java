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
package de.monticore.lang.math.math._symboltable.matrix;

import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;

import java.util.Optional;

/**
 * @author Sascha Schneiders
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
