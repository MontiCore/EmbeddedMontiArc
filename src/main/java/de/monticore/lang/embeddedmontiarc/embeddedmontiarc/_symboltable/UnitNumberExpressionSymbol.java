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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.lang.monticar.interfaces.TextualExpression;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberExpression;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.SymbolKind;

import java.text.DecimalFormat;

/**
 * Only used for getting expression value
 *
 * @author Sascha Schneiders
 */
public class UnitNumberExpressionSymbol extends CommonSymbol implements TextualExpression {
    protected ASTUnitNumberExpression unitNumberExpression;

    public UnitNumberExpressionSymbol() {
        super("", SymbolKind.KIND);
    }

    public UnitNumberExpressionSymbol(ASTUnitNumberExpression astUnitNumberExpression) {
        super("", SymbolKind.KIND);
        this.unitNumberExpression = astUnitNumberExpression;
    }


    @Override
    public String getTextualRepresentation() {
        String result = "";
        ASTNumberWithUnit num = unitNumberExpression.getNumberWithUnit();
        if (num.getComplexNumber().isPresent()) {
            result += String.format("%si%s", num.getComplexNumber().get().getRealNumber(), num.getComplexNumber().get().getImagineNumber());
        } else if (num.getNumber().isPresent()) {
            DecimalFormat format = new DecimalFormat("0.#");
            result += format.format(num.getNumber().get());
        } else {
            result += num.toString();
        }
        result += num.getUnit().toString();
        return result;
    }
}
