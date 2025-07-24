/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.symboltable.resolving.SymbolAdapter;

import java.util.Arrays;
import java.util.Collections;

/**
 */
public class PortSymbol2MathVariableDeclarationSymbol extends MathVariableDeclarationSymbol
        implements SymbolAdapter<EMAPortSymbol> {

    private final EMAPortSymbol adaptee;

    public PortSymbol2MathVariableDeclarationSymbol(EMAPortSymbol ps) {
        super(ps.getName(),
                new ASTRange(),
                Arrays.asList(1, 1),
                Collections.EMPTY_LIST);

        this.adaptee = ps;
    }

    public PortSymbol2MathVariableDeclarationSymbol(EMAPortSymbol ps, SIUnitRangesSymbol rangesSymbol) {
        super(ps.getName(),
                rangesSymbol.getRange(0),
                Arrays.asList(1, 1),
                Collections.EMPTY_LIST);

        this.adaptee = ps;
    }

    @Override
    public EMAPortSymbol getAdaptee() {
        return adaptee;
    }
}
