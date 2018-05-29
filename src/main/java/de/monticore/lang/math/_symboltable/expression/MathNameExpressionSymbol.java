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

/**
 * @author Sascha Schneiders
 */
public class MathNameExpressionSymbol extends MathValueExpressionSymbol implements IMathNamedExpression {

    protected String nameToResolveValue;

    public MathNameExpressionSymbol() {
        super();
    }

    public MathNameExpressionSymbol(String nameToResolveValue) {
        super();
        this.nameToResolveValue = nameToResolveValue;
    }

    public void setNameToAccess(String nameToAccess) {
        setNameToResolveValue(nameToAccess);
    }

    public String getNameToAccess() {
        return getNameToResolveValue();
    }

    public String getNameToResolveValue() {
        return nameToResolveValue;
    }

    public void setNameToResolveValue(String nameToResolveValue) {
        this.nameToResolveValue = nameToResolveValue;
    }

    @Override
    public String getTextualRepresentation() {
        return getNameToResolveValue();
    }

    public boolean isDottedName() {
        return nameToResolveValue.contains(".");
    }

    @Override
    public boolean isNameExpression() {
        return true;
    }
}
