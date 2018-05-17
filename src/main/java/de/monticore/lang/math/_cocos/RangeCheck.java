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
///**
// *
// *  ******************************************************************************
// *  MontiCAR Modeling Family, www.se-rwth.de
// *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
// *  All rights reserved.
// *
// *  This project is free software; you can redistribute it and/or
// *  modify it under the terms of the GNU Lesser General Public
// *  License as published by the Free Software Foundation; either
// *  version 3.0 of the License, or (at your option) any later version.
// *  This library is distributed in the hope that it will be useful,
// *  but WITHOUT ANY WARRANTY; without even the implied warranty of
// *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// *  Lesser General Public License for more details.
// *
// *  You should have received a copy of the GNU Lesser General Public
// *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
// * *******************************************************************************
// */
//
//package de.monticore.lang.math._cocos;
//
//
////import de.monticore.lang.math._ast.ASTAssignment;
//import de.monticore.lang.math._symboltable.*;
//import de.monticore.lang.monticar.ranges._ast.ASTRange;
//
//import de.se_rwth.commons.logging.Log;
//
//
//import javax.measure.unit.Unit;
//import de.monticore.lang.numberunit._ast.*;
///**
// * @author math-group
// *          Range check Cocos
// */
//public class RangeCheck extends AbstChecker /*implements MathASTAssignmentCoCo*/ {
//    /*
//    @Override
//    public void check(ASTAssignment assignment) {
//        boolean hasStart = false;
//        boolean hasEnd = false;
//        if(((MathVariableDeclarationSymbol) assignment.getSymbol().get()).getRange() != null)
//            hasStart = !((MathVariableDeclarationSymbol) assignment.getSymbol().get()).getRange().hasNoLowerLimit();
//        if(((MathVariableDeclarationSymbol) assignment.getSymbol().get()).getRange() != null)
//            hasEnd =   !((MathVariableDeclarationSymbol) assignment.getSymbol().get()).getRange().hasNoUpperLimit();
//        if (hasStart || hasEnd) {
//
//            MathVariableDeclarationSymbol tmp = (MathVariableDeclarationSymbol) assignment.getSymbol().get();
//            MathValue value = tmp.getValue();
//            // range of the value of the symbol
//            ASTRange rangeFromExp = new ASTRange();
//
//            if (value instanceof MathExpression) {
//                if (((MathExpression) value).getOp().equals(Operator.Assign)) {
//                    tmp = ((MathVariableDeclarationSymbol)assignment.getMathExpression().get().getSymbol().get());
//                    MathValue speichereGetValue = (((MathVariableDeclarationSymbol)assignment.getMathExpression().get().getSymbol().get()).getValue());//(MathExpression)value).getOperands().get(0).getValue();
//                    tmp = ((MathValueReference) ((MathExpression) value).getOperands().get(0).getValue()).getReferencedSymbol();
//                    value = speichereGetValue;
//                }
//            }
//
//            //check if Range is correct
//            // e.g. (3:2) is not allowed b/c 3>2
//            if(!correctRange(tmp)) {
//                Log.error("0xMATH22 Wrong Range at Assignment");
//            }
//
//            // check Ranges for Variables w/ Dimension (-> isMatrix == true)
//
//            if (assignment.typeIsPresent() && assignment.getType().get().dimIsPresent()) {
//                    if (value instanceof JSMatrix) {
//                        rangeFromExp = rangeOfMatrix(tmp);
//                        if (!compareRanges(hasStart, hasEnd, tmp, rangeFromExp)) {
//                            Log.error("0xMATH24 Matrix Range does not fit");
//                        }
//                    }
//                    // else it is a MathExpression or MathValueReference and has to be calculated
//                    else {
//                        rangeFromExp = rangeCheck(tmp, true);
//                        if (!compareRanges(hasStart, hasEnd, tmp, rangeFromExp)) {
//                            Log.error("0xMATH24 Matrix Range does not fit");
//                        }
//                    }
//            }
//
//            // check Ranges for Variables w/o Dimension (-> isMatrix == false)
//            else if (value instanceof JSValue) {
//                rangeFromExp.setStart(ASTUnitNumber.valueOf(((JSValue) value).getRealNumber(), Unit.ONE));
//                rangeFromExp.setEnd(ASTUnitNumber.valueOf(((JSValue) value).getRealNumber(), Unit.ONE));
//                if (!compareRanges(hasStart, hasEnd, tmp, rangeFromExp))
//                    Log.error("0xMATH23 Range does not fit");
//
//            }
//            // else it is a MathExpression or MathValueReference and has to be calculated
//            else {
//                rangeFromExp = rangeCheck(tmp, false);
//                if(!compareRanges(hasStart, hasEnd, tmp, rangeFromExp))
//                    Log.error("0xMATH23 Range does not fit");
//            }
//
//
//        }
//    }
//
//    public boolean compareRanges(boolean hasStart, boolean hasEnd, MathVariableDeclarationSymbol tmp, ASTRange rangeFromExp) {
//        if (hasStart && hasEnd) {
//            if(!rangeFromExp.startIsPresent() || !rangeFromExp.endIsPresent()) return false;
//            else {
//                double tmpStartRange = tmp.getRange().getStartValue().doubleValue();
//                double tmpEndRange = tmp.getRange().getEndValue().doubleValue();
//                // if there is a unit in tmp's range, we have to
//                // save the startRange in standardUnit to make right comparision
//                if(tmp.getRange().hasStartUnit()) {
//                    tmpStartRange = tmp.getRange().getStartUnit().toStandardUnit().convert(tmp.getRange().getStartValue().doubleValue());
//                }
//                else if (tmp.getRange().hasEndUnit()) {
//                    tmpStartRange = tmp.getRange().getEndUnit().toStandardUnit().convert(tmp.getRange().getStartValue().doubleValue());
//                }
//                else if (tmp.getRange().stepIsPresent()) {
//                    if(tmp.getRange().hasEndUnit())
//                        tmpStartRange = tmp.getRange().getStepUnit().toStandardUnit().convert(tmp.getRange().getStartValue().doubleValue());
//                }
//                // same for endRange
//                if(tmp.getRange().hasEndUnit()) {
//                    tmpEndRange = tmp.getRange().getEndUnit().toStandardUnit().convert(tmp.getRange().getEndValue().doubleValue());
//                }
//                else if (tmp.getRange().hasStartUnit()){
//                    tmpEndRange = tmp.getRange().getStartUnit().toStandardUnit().convert(tmp.getRange().getEndValue().doubleValue());
//                }
//                else if (tmp.getRange().stepIsPresent()) {
//                        if(tmp.getRange().hasStepUnit()) {
//                            tmpEndRange = tmp.getRange().getStepUnit().toStandardUnit().convert(tmp.getRange().getEndValue().doubleValue());
//                        }
//                }
//                if (tmpStartRange > (rangeFromExp.getStartValue().doubleValue()) || tmpEndRange < (rangeFromExp.getEndValue().doubleValue())) {
//                    return false; }
//                return true;
//            }
//        } else if (!hasStart && hasEnd) {
//            if (!rangeFromExp.endIsPresent()) return false;
//            else {
//                double tmpEndRange = tmp.getRange().getEndValue().doubleValue();
//                // if there is a unit in tmp's range, we have to
//                // save the EndRange in standardUnit to make right comparision
//                if(tmp.getRange().hasEndUnit())
//                    tmpEndRange = tmp.getRange().getEndUnit().toStandardUnit().convert(tmp.getRange().getEndValue().doubleValue());
//                if(tmp.getRange().stepIsPresent()) {
//                    if(tmp.getRange().hasStepUnit()) {
//                        tmpEndRange = tmp.getRange().getStepUnit().toStandardUnit().convert(tmp.getRange().getEndValue().doubleValue());
//                    }
//                }
//                if (tmpEndRange < (rangeFromExp.getEndValue().doubleValue()))
//                    return false;
//                return true;
//            }
//        } else //(hasStart && !hasEnd)
//        {
//            if (!rangeFromExp.startIsPresent()) return false;
//            else {
//                double tmpStartRange = tmp.getRange().getEndValue().doubleValue();
//                // if there is a unit in tmp's range, we have to
//                // save the startRange in standardUnit to make right comparision
//                if(tmp.getRange().hasStartUnit())
//                    tmpStartRange = tmp.getRange().getStartUnit().toStandardUnit().convert(tmp.getRange().getStartValue().doubleValue());
//                if(tmp.getRange().stepIsPresent()) {
//                    if(tmp.getRange().hasStepUnit()) {
//                        tmpStartRange = tmp.getRange().getStepUnit().toStandardUnit().convert(tmp.getRange().getStartValue().doubleValue());
//                    }
//                }
//                if (tmpStartRange > (rangeFromExp.getStartValue().doubleValue()))
//                    return false;
//                return true;
//            }
//        }
//
//    }
//
//
//    // checks if Range of the Symbol is semantical correct
//
//    public boolean correctRange(MathVariableDeclarationSymbol symbol) {
//        if(symbol.getRange() != null) {
//            if (symbol.getRange().startIsPresent() && symbol.getRange().endIsPresent()) {
//                if (symbol.getRange().getStartValue().doubleValue()  > (symbol.getRange().getEndValue().doubleValue())) {
//                    return false;
//                }
//            }
//        }
//        return true;
//    }
//*/
//}
//
