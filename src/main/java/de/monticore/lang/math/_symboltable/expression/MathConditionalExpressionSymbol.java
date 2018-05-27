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
package de.monticore.lang.math._symboltable.expression;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class MathConditionalExpressionSymbol extends MathExpressionSymbol {
    protected Optional<MathExpressionSymbol> condition = Optional.empty();
    protected List<MathExpressionSymbol> bodyExpressions = new ArrayList<>();

    public Optional<MathExpressionSymbol> getCondition() {
        return condition;
    }

    public void setCondition(MathExpressionSymbol condition) {
        this.condition = Optional.of(condition);
    }

    public List<MathExpressionSymbol> getBodyExpressions() {
        return bodyExpressions;
    }

    public void setBodyExpressions(List<MathExpressionSymbol> bodyExpressions) {
        this.bodyExpressions = bodyExpressions;
    }

    public void addBodyExpression(MathExpressionSymbol bodyExpression) {
        bodyExpressions.add(bodyExpression);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";

        if (condition.isPresent())
            result += "(" + condition.get().getTextualRepresentation() + ")";
        if (bodyExpressions.size() > 0) {
            result += "{\n";
            for (MathExpressionSymbol symbol : bodyExpressions) {
                result += symbol.getTextualRepresentation() + ";\n";
            }
            result += "}\n";
        }
        return result;
    }
}
