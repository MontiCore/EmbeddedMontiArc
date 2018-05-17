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
//package de.monticore.lang.math.math._cocos;
//
////import de.monticore.lang.math.math._ast.ASTAssignment;
//import de.monticore.lang.math.math._symboltable.*;
//import de.se_rwth.commons.logging.Log;
//
//import javax.measure.unit.Unit;
//import java.util.Arrays;
//import java.util.List;
//
//
//public class DimensionEquals extends AbstChecker /*implements MathASTAssignmentCoCo */{
//    /*@Override
//    public void check(ASTAssignment astAssignment) {
//        List<Integer> dims;
//        Unit unit = Unit.ONE;
//        MathVariableDeclarationSymbol tmp = (MathVariableDeclarationSymbol) astAssignment.getSymbol().get();
//        MathValue test = tmp.getValue();
//        Log.debug(tmp.toString(),"tmp1:");
//        Log.debug(test.toString(),"test:");
//        if (test instanceof MathExpression) {
//            if (((MathExpression) test).getOp().equals(Operator.Assign)) {
//                tmp = ((MathValueReference) ((MathExpression) test).getOperands().get(0).getValue()).getReferencedSymbol();
//            }
//        }
//
//        Log.debug(tmp.toString(),"tmp2:");
//        //check if the Range is correct
//        if (tmp.getRange() != null) {
//            unit = checkRange(tmp);
//
//        }
//        //check if the Unit of the Range and the assigned value are compatible
//        if (!unit.isCompatible(checkUnits(test))) {
//            Log.debug(unit.toString(), "Unit1:");
//            Log.debug(checkUnits(test).toString(), "Unit2:");
//            Log.error("0xMATH21 Invalid Units at assignment");
//        }
//        //get the Dimension of the assigned value
//        if (test instanceof JSMatrix) {
//            dims = Arrays.asList(((JSMatrix) test).getRowDimension(), ((JSMatrix) test).getColDimension());
//        } else if (test instanceof JSValue) {
//            dims = Arrays.asList(1, 1);
//        } else {
//            dims = checkDimension(tmp);
//        }
//
//        checkDimHelper(astAssignment, tmp.getDimensions(), dims);
//
//    }
//
//    /**
//     * calculates the Unit of the Range and checks them
//     *
//     * @param symbol MathVariableDeclarationSymbol we want to check the range from
//     * @return Unit of the Range if it is consistant
//     */ /*
//    public Unit checkRange(MathVariableDeclarationSymbol symbol) {
//        //Unit is only available if there is a Start and endpoint present
//        if (symbol.getRange().startIsPresent() && symbol.getRange().endIsPresent()) {
//            Unit startUnit = symbol.getRange().getStartUnit();
//            Unit stepUnit = null;
//            Log.debug(startUnit.toString(), "StartUnit:");
//            if (symbol.getRange().stepIsPresent()) {
//                stepUnit = symbol.getRange().getStepUnit();
//                Log.debug(stepUnit.toString(), "StepUnit:");
//            }
//            Unit endUnit = symbol.getRange().getEndUnit();
//            if (!startUnit.isCompatible(endUnit) && (!startUnit.equals(Unit.ONE))) {
//                Log.error("0xMATH20 incompatible Units in Range");
//            } else if (stepUnit != null) {
//                if (!endUnit.isCompatible(stepUnit)) {
//                    Log.error("0xMATH20 incompatible Units in Range");
//                }
//            }
//            return endUnit;
//        } else if (symbol.getRange().startIsPresent()) {
//            return symbol.getRange().getStartUnit();
//        }
//
//        Log.debug(symbol.toString(), "No Unit present:");
//        //if not we return standard Unit
//        return Unit.ONE;
//    }
//
//    /**
//     * checks if the assigned value has the same Dimension as Defined
//     *
//     * @param astAssignment assignment
//     * @param dim           the defined Dimensions
//     * @param dims          the calculated Dimensions
//     * @return Unit of the Range if it is consistant
//     */ /*
//    public void checkDimHelper(ASTAssignment astAssignment, List<Integer> dim, List<Integer> dims) {
//        if (astAssignment.typeIsPresent()) {
//            if (astAssignment.getType().get().dimIsPresent()) {
//                if (dim.size() == 1) {
//                    if (dim.get(0) != dims.get(1)) {
//                        Log.error("0xMATH02 Wrong Dimension at assignment");
//                    }
//                } else {
//                    for (int i = 0; i < dim.size(); i++) {
//                        if (dim.get(i) != dims.get(i)) {
//                            Log.error("0xMATH02 Wrong Dimension at assignment");
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//    } */
//}
