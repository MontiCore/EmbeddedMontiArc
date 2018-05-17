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

import de.monticore.lang.math._ast.ASTAssignmentType;
import de.monticore.lang.math._ast.ASTDimension;
import de.monticore.lang.math._ast.ASTMathArithmeticExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonDimensionElement;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.types2._ast.ASTElementType;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MathValueType extends MathExpressionSymbol {
    protected ASTElementType type;
    protected List<String> properties = new ArrayList<>();
    protected List<MathExpressionSymbol> dimensions = new ArrayList<>();

    public MathValueType() {

    }

    public void addDimension(MathExpressionSymbol dimension) {
        dimensions.add(dimension);
    }

    public List<MathExpressionSymbol> getDimensions() {
        return dimensions;
    }

    public List<String> getProperties() {
        return properties;
    }

    public void setProperties(List<String> properties) {
        this.properties = properties;
    }

    public void setDimensions(List<MathExpressionSymbol> dimensions) {
        this.dimensions = dimensions;
    }

    public ASTElementType getType() {
        return type;
    }

    public void setType(ASTElementType type) {
        this.type = type;
    }

    public boolean isRationalType() {
        return type.isIsRational();
    }


    public boolean isComplexType() {
        return type.isIsComplex();
    }

    public boolean isStatic() {
        return properties.contains("static");
    }

    @Override
    public boolean isMathValueTypeExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";

        //result += type.toString();

        if (isRationalType())
            result += "Q";
        else if (isComplexType()) {
            result += "C";
        }
        if (type.getRange().isPresent()) {
            result += type.getRange().get().toString();
        }
        if (dimensions.size() > 0) {
            int counter = 0;
            result += "^{";
            for (MathExpressionSymbol dimension : dimensions) {
                result += dimension.getTextualRepresentation();
                ++counter;
                if (dimensions.size() > counter)
                    result += ", ";
            }
            result += "}";
        }
        return result;
    }

    public static MathValueType convert(ASTAssignmentType type) {
        MathValueType mathValueType = new MathValueType();

        mathValueType.setProperties(type.getMatrixProperty());

        if (type.commonMatrixTypeIsPresent()) {
            //TODO type
            ASTCommonMatrixType commonMatrixType = type.getCommonMatrixType().get();
            mathValueType.setType(commonMatrixType.getElementType());

            for (ASTCommonDimensionElement astCommonDimensionElement : commonMatrixType.getCommonDimension().getCommonDimensionElements()) {
                if (astCommonDimensionElement.getName().isPresent()) {
                    mathValueType.addDimension(new MathNameExpressionSymbol(astCommonDimensionElement.getName().get()));
                } else if (astCommonDimensionElement.getUnitNumber().isPresent()) {
                    mathValueType.addDimension(new MathNumberExpressionSymbol(astCommonDimensionElement.getUnitNumber().get().getNumber().get()));
                }
            }
        } else if (type.getElementType().isPresent()) {
            //TODO type
            mathValueType.setType(type.getElementType().get());

            if (type.getDim().isPresent()) {
                ASTDimension astDimension = type.getDim().get();
                for (ASTMathArithmeticExpression astMathArithmeticExpression : astDimension.getMathArithmeticExpressions()) {
                    mathValueType.addDimension((MathExpressionSymbol) astMathArithmeticExpression.getSymbol().get());
                }
            }
        }

        return mathValueType;
    }
}
