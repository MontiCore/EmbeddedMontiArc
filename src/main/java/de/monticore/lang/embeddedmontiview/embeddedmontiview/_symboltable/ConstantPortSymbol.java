/**
 * ******************************************************************************
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
///*
// * ******************************************************************************
// * MontiCore Language Workbench, www.monticore.de
// * Copyright (c) 2017, MontiCore, All rights reserved.
// *
// * This project is free software; you can redistribute it and/or
// * modify it under the terms of the GNU Lesser General Public
// * License as published by the Free Software Foundation; either
// * version 3.0 of the License, or (at your option) any later version.
// * This library is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// * Lesser General Public License for more details.
// *
// * You should have received a copy of the GNU Lesser General Public
// * License along with this project. If not, see <http://www.gnu.org/licenses/>.
// * ******************************************************************************
// */
//
//package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;
//
//
//import de.monticore.lang.embeddedmontiview.embeddedmontiview.unit.constant.*;
//
////import de.monticore.literals.literals._ast.*;
//import de.monticore.lang.monticar.literals2._ast.*;
//import de.se_rwth.commons.logging.Log;
//import org.jscience.mathematics.number.Rational;
//
//import javax.measure.unit.Unit;
//
//import siunit.monticoresiunit.si._ast.ASTUnitNumber;
//
///**
// * The ConstantPortSymbol is a port which has a constant value assigned and is used
// * by a ConstantConnector to connect this value to other ports.
// *
// * @author Sascha Schneiders
// */
//public class ConstantPortSymbol extends PortSymbol {
//    EMAConstantValue constantValue;
//
//    /**
//     * use this constructor for automatic naming of constant ports
//     */
//    public ConstantPortSymbol() {
//        super(ConstantPortSymbol.getNextConstantPortName());
//    }
//
//    public ConstantPortSymbol(String name) {
//        super(name);
//    }
//
//
//    public EMAConstantValue getConstantValue() {
//        return constantValue;
//    }
//
//
//    public void setConstantValue(EMAConstantValue value) {
//        Log.debug("" + value.getValue().toString(), "value setting");
//        this.constantValue = value;
//    }
//
//
//    /**
//     * initializes ConstantPort from a UnitNumberLiteral
//     */
//    public void initConstantPortSymbol(ASTUnitNumber si_unit) {
//        Unit unit = si_unit.getUnit().get();
//        Rational rational = si_unit.getNumber().get();
//
//        setConstantValue(new EMAConstantSIUnit(rational, unit));
//    }
//
//    /**
//     * initializes ConstantPort from a BooleanLiteral
//     */
//    public void initConstantPortSymbol(ASTBooleanLiteral astBooleanLiteral) {
//        setConstantValue(new EMAConstantBoolean(astBooleanLiteral.getSource() == 1));
//    }
//
//    private static int lastID = 1;
//
//    public static String getNextConstantPortName() {
//        return "CONSTANTPORT" + lastID++;
//    }
//
//    @Override
//    public boolean isConstant() {
//        return true;
//    }
//
//    public static ConstantPortSymbol createConstantPortSymbol(de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTConnector node) {
//        ConstantPortSymbol constantPortSymbol = new ConstantPortSymbol();
//
//        if (node.getUnitNumberResolution().isPresent()) {
//            constantPortSymbol.initConstantPortSymbol(node.getUnitNumberResolution().get().getUnitNumber().get());
//        } else if (node.getBoolLiteral().isPresent()) {
//            constantPortSymbol.initConstantPortSymbol(node.getBoolLiteral().get());
//        }
//
//        return constantPortSymbol;
//    }
//}
