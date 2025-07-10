/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import de.monticore.lang.math._matrixprops.MatrixProperties;

import java.util.ArrayList;

/**
 * Symbol represents a MathValueSymbol which consists of a type and a mathexpression that determines its value.
 *
 */
public class MathValueSymbol extends MathValueExpressionSymbol {

    protected MathValueType type;
    protected MathExpressionSymbol value;
    protected ArrayList<MatrixProperties> matrixProperties;

    public MathValueSymbol(String name) {
        super(name);
    }

    public MathValueType getType() {
        return type;
    }

    public void setType(MathValueType type) {
        this.type = type;
    }

    public MathExpressionSymbol getValue() {
        return value;
    }

    public void setValue(MathExpressionSymbol value) {
        this.value = value;
    }

    public ArrayList<MatrixProperties> getMatrixProperties() {
        return matrixProperties;
    }

    public void setMatrixProperties(ArrayList<MatrixProperties> matrixProperties) {
        this.matrixProperties = matrixProperties;
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        if (type != null)
            result += type.getTextualRepresentation() + " ";

        result += getFullName();

        if (value != null)
            result += " = " + getValue().getTextualRepresentation();
        return result;
    }

    public boolean isMatrixValueSymbol() {
        return false;
    }

    @Override
    public boolean isAssignmentDeclarationExpression() {
        return true;
    }


    public MathExpressionSymbol getAssignedMathExpressionSymbol() {
        if (value != null)
            return value;
        return this;
    }
}
