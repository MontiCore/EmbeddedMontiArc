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

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
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
