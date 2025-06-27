/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter;

import java.util.Arrays;
import java.util.Collections;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.symboltable.resolving.SymbolAdapter;

/**
 * Created by MichaelvonWenckstern on 27.01.2017.
 */
public class PortArraySymbol2MathVariableDeclarationSymbol extends MathVariableDeclarationSymbol
        implements SymbolAdapter<EMAPortArraySymbol> {

    private final EMAPortArraySymbol adaptee;

    public PortArraySymbol2MathVariableDeclarationSymbol(EMAPortArraySymbol ps) {
        super(ps.getName(),
                new ASTRange(),
                Arrays.asList(1, ps.getDimension()),
                Collections.EMPTY_LIST);

        this.adaptee = ps;
    }

    public PortArraySymbol2MathVariableDeclarationSymbol(EMAPortArraySymbol ps, SIUnitRangesSymbol rangesSymbol) {
        super(ps.getName(),
                rangesSymbol.getRange(0),
                Arrays.asList(1, ps.getDimension()),
                Collections.EMPTY_LIST);

        this.adaptee = ps;
    }

    @Override
    public EMAPortArraySymbol getAdaptee() {
        return adaptee;
    }

}
