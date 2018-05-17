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
package de.monticore.lang.math.math._symboltable.expression;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class MathConditionalExpressionsSymbol extends MathExpressionSymbol {
    protected MathConditionalExpressionSymbol ifConditionalExpression;
    protected List<MathConditionalExpressionSymbol> ifElseConditionalExpressions = new ArrayList<>();
    protected Optional<MathConditionalExpressionSymbol> elseConditionalExpression = Optional.empty();

    public MathConditionalExpressionSymbol getIfConditionalExpression() {
        return ifConditionalExpression;
    }

    public void setIfConditionalExpression(MathConditionalExpressionSymbol ifConditionalExpression) {
        this.ifConditionalExpression = ifConditionalExpression;
    }

    public List<MathConditionalExpressionSymbol> getIfElseConditionalExpressions() {
        return ifElseConditionalExpressions;
    }

    public void setIfElseConditionalExpressions(List<MathConditionalExpressionSymbol> ifElseConditionalExpressions) {
        this.ifElseConditionalExpressions = ifElseConditionalExpressions;
    }

    public Optional<MathConditionalExpressionSymbol> getElseConditionalExpression() {
        return elseConditionalExpression;
    }

    public void setElseConditionalExpression(MathConditionalExpressionSymbol elseConditionalExpression) {
        this.elseConditionalExpression = Optional.of(elseConditionalExpression);
    }

    public void addElseIfConditionalExpression(MathConditionalExpressionSymbol elseIfConditionalExpressionSymbol) {
        ifElseConditionalExpressions.add(elseIfConditionalExpressionSymbol);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "If:";

        result += ifConditionalExpression.getTextualRepresentation();
        if (ifElseConditionalExpressions.size() > 0)
            result += "Else If:";
        for (MathConditionalExpressionSymbol symbol : ifElseConditionalExpressions) {
            result += symbol.getTextualRepresentation();
        }
        if (elseConditionalExpression.isPresent())
            result += "Else:" + elseConditionalExpression.get().getTextualRepresentation();

        return result;
    }

    @Override
    public boolean isConditionalsExpression() {
        return true;
    }

}
