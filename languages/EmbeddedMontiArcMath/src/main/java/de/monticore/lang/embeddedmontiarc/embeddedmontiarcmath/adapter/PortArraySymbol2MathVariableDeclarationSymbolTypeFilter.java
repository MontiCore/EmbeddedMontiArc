/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.math._symboltable.MathVariableDeclarationSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.TransitiveAdaptedResolvingFilter;

/**
 * Created by MichaelvonWenckstern on 02.02.2017.
 */
public class PortArraySymbol2MathVariableDeclarationSymbolTypeFilter
        extends TransitiveAdaptedResolvingFilter<MathVariableDeclarationSymbol> {

    public PortArraySymbol2MathVariableDeclarationSymbolTypeFilter() {
        super(EMAPortArraySymbol.KIND,
                MathVariableDeclarationSymbol.class,
                MathVariableDeclarationSymbol.KIND);
    }

    @Override
    public Symbol translate(Symbol adaptee) {
        assert adaptee instanceof EMAPortArraySymbol;
        if (((EMAPortArraySymbol) adaptee).getTypeReference().getReferencedSymbol() instanceof SIUnitRangesSymbol)
            return new PortArraySymbol2MathVariableDeclarationSymbol((EMAPortArraySymbol) adaptee, (SIUnitRangesSymbol) ((EMAPortArraySymbol) adaptee).getTypeReference().getReferencedSymbol());
        else {
            MontiCarTypeSymbol jTypeSymbol = (MontiCarTypeSymbol) ((EMAPortArraySymbol) adaptee).getTypeReference().getReferencedSymbol();
            System.out.println("Adaption of: " + jTypeSymbol.toString());
            return new PortArraySymbol2MathVariableDeclarationSymbol((EMAPortArraySymbol) adaptee);
        }

    }
}
