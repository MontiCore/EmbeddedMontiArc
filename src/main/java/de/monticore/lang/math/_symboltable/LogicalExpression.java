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
 * Type for logical expressions
 */
public class LogicalExpression  implements MathValue{
    /** operator which connects the operands */
    private Operator op;

    /** list of operands */
    private ArrayList<MathVariableDeclarationSymbol> operands;

    /**
     * creates an empty object
     */
    public LogicalExpression(){
        this.op = null;
        this.operands = new ArrayList<>();
    }

    /**
     * creates a copy of this object
     *
     * @param op operator
     * @param operands
     */
    public LogicalExpression(Operator op, ArrayList<MathVariableDeclarationSymbol> operands){
        this.op = op;
        this.operands = operands;
    }

    /**
     * @return the operator of this logical expression
     */
    public Operator getOp(){
        return this.op;
    }

    /**
     * replace the operator of this object
     *
     * @param op new operator
     */
    public void setOp(Operator op){
        this.op = op;
    }

    /**
     * set a new list of operands
     *
     * @param operands new operands
     */
    public void setOperands(ArrayList<MathVariableDeclarationSymbol> operands){
        this.operands = operands;
    }

    /**
     * @return the operands list of this logical expression
     */
    public ArrayList<MathVariableDeclarationSymbol> getOperands(){
        return this.operands;
    }

    /**
     * add a operand to the end of the list
     *
     * @param operand new operand
     */
    public void addOperand(MathVariableDeclarationSymbol operand) {
        this.operands.add(operand);
    }

    /**
     * create a string based representation out of this logical expression
     *
     * @return string based representation
     */
    @Override
    public String toString(){
        if(operands.size() == 1){
            return toStringOneOperand();
        }else if(operands.size() == 2){
            return toStringTwoOperand();
        }else{
            // System.out.print("unknown amount of operands");

            return null;
        }
    }

    /**
     * create a string based representation out of this logical expression with 1 operand
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
            output+=op;
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

    @Override
    public boolean isIncompUnit() {
        return false;
    }

    @Override
    public void setIncompUnit(boolean x) {
    }
    @Override
    public Unit getUnit() {
        return Unit.ONE;
    }

    @Override
    public void setUnit(Unit unit) {

    }

}
