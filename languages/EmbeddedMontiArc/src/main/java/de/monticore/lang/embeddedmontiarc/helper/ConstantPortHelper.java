/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.helper;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortBuilder;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantBoolean;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.unit.constant.EMAConstantSIUnit;
import de.monticore.literals.literals._ast.ASTBooleanLiteral;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import javax.measure.unit.Unit;

import static de.monticore.lang.embeddedmontiarc.helper.EMATypeHelper.initTypeRefGeneralType;
import static de.monticore.numberunit.Rationals.doubleToRational;

/**
 * The ConstantPortHelper manages constant Port names and sets the constantValue from the AST elements. Constant Ports are used
 * by constant Connectors to connect their value to other ports.
 *
 */
public class ConstantPortHelper {
    private static int lastID = 1;

    public static String getNextConstantPortName() {
        return "CONSTANTPORT" + lastID++;
    }

    public static void resetLastID() {
        lastID = 1;
    }

    /**
     * converts a UnitNumberLiteral to EMAConstantValue for a constant Port
     */
    public static EMAConstantSIUnit getConstantValue(ASTNumberWithUnit si_unit) {
        Unit unit = si_unit.getUnit();
        Rational rational = doubleToRational(si_unit.getNumber().get());

        return new EMAConstantSIUnit(rational, unit);
    }

    /**
     * converts a BooleanLiteral to EMAConstantValue for a constant Port
     */
    public static EMAConstantBoolean getConstantValue(ASTBooleanLiteral astBooleanLiteral) {
        return new EMAConstantBoolean(astBooleanLiteral.getSource() == 1);
    }

    public static EMAPortSymbol createConstantPortSymbol(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector node, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        EMAPortBuilder builder = EMAPortSymbol.builder();
        builder.setName(getNextConstantPortName());
        builder.setDirection(EMAPortSymbol.INCOMING);

        if (node.getUnitNumberResolutionOpt().isPresent()) {
            builder.setConstantValue(getConstantValue(node.getUnitNumberResolution().getNumberWithUnit()));
            String typeName = "UnitNumberResolution";
            builder.setTypeReference(initTypeRefGeneralType(typeName, symbolTableCreator));
        } else if (node.getBoolLiteralOpt().isPresent()) {
            builder.setConstantValue(getConstantValue(node.getBoolLiteral()));
            String typeName = "B";
            builder.setTypeReference(initTypeRefGeneralType(typeName, symbolTableCreator));
        } else {
            Log.info("Case not handled", "ConstantPortInit");
        }

        return builder.build();
    }
}
