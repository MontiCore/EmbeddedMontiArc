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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantBoolean;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantSIUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantValue;
import de.monticore.literals.literals._ast.ASTBooleanLiteral;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;

import static de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper.initTypeRefGeneralType;
import static de.monticore.numberunit.Rationals.doubleToRational;

//import de.monticore.literals.literals._ast.*;

/**
 * The EMAConstantPortSymbol is a port which has a constant value assigned and is used
 * by a ConstantConnector to connect this value to other ports.
 *
 * @author Sascha Schneiders
 */
public class EMAConstantPortSymbol extends EMAPortSymbol {
    EMAConstantValue constantValue;

    /**
     * use this constructor for automatic naming of constant ports
     */
    public EMAConstantPortSymbol() {
        super(EMAConstantPortSymbol.getNextConstantPortName());
        setDirection(true);
    }

    public EMAConstantPortSymbol(String name) {
        super(name);
        setDirection(true);
    }


    public EMAConstantValue getConstantValue() {
        return constantValue;
    }


    public void setConstantValue(EMAConstantValue value) {
        Log.debug("" + value.getValue().toString(), "value setting");
        this.constantValue = value;
    }


    /**
     * initializes ConstantPort from a UnitNumberLiteral
     */
    public void initConstantPortSymbol(ASTNumberWithUnit si_unit) {
        Unit unit = si_unit.getUnit();
        Rational rational = doubleToRational(si_unit.getNumber().get());

        setConstantValue(new EMAConstantSIUnit(rational, unit));
    }

    /**
     * initializes ConstantPort from a BooleanLiteral
     */
    public void initConstantPortSymbol(ASTBooleanLiteral astBooleanLiteral) {
        setConstantValue(new EMAConstantBoolean(astBooleanLiteral.getSource() == 1));
    }

    private static int lastID = 1;

    public static String getNextConstantPortName() {
        return "CONSTANTPORT" + lastID++;
    }

    public static void resetLastID() {
        lastID = 1;
    }

    @Override
    public boolean isConstant() {
        return true;
    }

    public static EMAConstantPortSymbol createConstantPortSymbol(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector node, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        EMAConstantPortSymbol emaConstantPortSymbol = new EMAConstantPortSymbol();

        if (node.getUnitNumberResolutionOpt().isPresent()) {
            emaConstantPortSymbol.initConstantPortSymbol(node.getUnitNumberResolution().getNumberWithUnit());
            String typeName;
            typeName = "UnitNumberResolution";
            emaConstantPortSymbol.setTypeReference(initTypeRefGeneralType(typeName, symbolTableCreator));
        } else if (node.getBoolLiteralOpt().isPresent()) {
            emaConstantPortSymbol.initConstantPortSymbol(node.getBoolLiteral());
            String typeName;
            typeName = "B";
            emaConstantPortSymbol.setTypeReference(initTypeRefGeneralType(typeName, symbolTableCreator));
        } else {
            Log.info("Case not handled", "ConstantPortInit");
        }

        return emaConstantPortSymbol;
    }
}
