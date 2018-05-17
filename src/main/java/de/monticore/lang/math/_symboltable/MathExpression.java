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
package de.monticore.lang.math._symboltable;



import javax.measure.unit.Unit;
import java.util.ArrayList;

/**
 * @author math-group
 *
 * JScience Math expression type
 */

public class MathExpression implements MathValue{
    /** operator which connects the operands like +,-,*,... */
    private Operator op;

    /** a list of operands, generally of size 1-2. Here we use
     * unary (like ' (transpose), ++,-- (increment), ..)
     * binary (like +, -, *, common operators from algebra
     *
     * for instance in 1 + 2 are 1 and 2 the operands.
     */
    private ArrayList<MathVariableDeclarationSymbol> operands;

    /** every valid math expression has one common unit */
    private Unit unit=Unit.ONE;

    /** flag for incompatible units */
    private boolean incompUnit = false;

    /**
     * initialize a math expression with an empty operands list and operator
     */
    public MathExpression() {
        this.op = null;
        this.operands = new ArrayList<MathVariableDeclarationSymbol>();
    }

    /**
     * initialize a math expression with the values given by the arguments
     * @param op operator
     * @param operands list of operands
     */
    public MathExpression(Operator op, ArrayList<MathVariableDeclarationSymbol> operands) {
        this.op = op;
        this.operands = operands;
    }

    /**
     * set a (new) operator
     * @param op operator
     */
    public void setOp(Operator op){
        this.op = op;
    }

    /**
     * @return current operator
     */
    public Operator getOp() {
        return this.op;
    }

    /**
     * set a new list of operands
     * @param operands list of operands
     */
    public void setOperands(ArrayList<MathVariableDeclarationSymbol> operands) {
        this.operands = operands;
    }

    /**
     * @return get the current list of operands
     */
    public ArrayList<MathVariableDeclarationSymbol> getOperands() {
        return this.operands;
    }

    /**
     * add a new operand to the current list of operands
     * @param operand new operand
     */
    public void addOperand(MathVariableDeclarationSymbol operand) {
        this.operands.add(operand);
    }

    /**
     * set the unit for the math expression
     * @param unit (jscience unit)
     */
    public void setUnit(Unit unit){
        this.unit = unit;
    }
    /**
     * check if the units are incompatible
     *
     * @return TRUE if the units are incompatible, otherwise FALSE
     */
    @Override
    public boolean isIncompUnit() {
        return incompUnit;
    }
    /**
     * @param x set TRUE if the math value/expression has incompatible units, otherwise FALSE
     */
    @Override
    public void setIncompUnit(boolean x) {
        this.incompUnit = x;
    }

    /**
     * @return get the unit of this math expression
     */
    public Unit getUnit() {
        return unit;
    }
    /**
     * convert the JScience based math expression to a String based one
     * only two operands are supported.
     *
     * @return the String {@link String} based representation of the JScience math expression
     *
     * for instance: (2 + 3) is a valid String representation
     * 2 operands are connected by the operator and surrounded by parenthesis
     */
    @Override
    public String toString() {
        String output = new String();

        if(operands.size() == 1) {
            return toStringOneOperand();
        } else if(operands.size() == 2) {
            return  toStringTwoOperand();
        }

       return output;
    }

    /**
     * create a string based representation out of this math expression with 1 operand
     *
     * @return string based representation
     */
    public String toStringOneOperand(){
        String output = new String();

        if(operands.size() == 1) {
            if(operands.get(0).getValue() != null) {
                output = operands.get(0).getValue().toString();
            } else {
                output = operands.get(0).toString();
            }
            output+=op.toString();
        }

        return output;
    }
    /**
     * create a string based representation out of this logical expression with 2 operands
     *
     * @return string based representation
     */
    public String toStringTwoOperand(){
        String output = new String();

        if(operands.size() == 2) {
            if(operands.get(0).getValue()!= null && operands.get(1).getValue() != null)
                output = "(" + operands.get(0).getValue().toString() + op.toString() + operands.get(1).getValue().toString() + ")";
            else{
                if(operands.get(0).getValue()!=null){
                    output = "(" + operands.get(0).getValue().toString() + op.toString() + operands.get(1).toString() + ")";
                }
                else output = "(" + operands.get(0).toString() + op.toString() + operands.get(1).getValue().toString() + ")";
            }
        }

        return output;
    }
}
