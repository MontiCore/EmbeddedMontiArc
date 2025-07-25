/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.TransitiveAdaptedResolvingFilter;

public class PortArraySymbol2IODeclarationSymbolTypeFilter extends TransitiveAdaptedResolvingFilter<IODeclarationSymbol> {

    public PortArraySymbol2IODeclarationSymbolTypeFilter() {
        super(EMAPortArraySymbol.KIND,
                IODeclarationSymbol.class,
                IODeclarationSymbol.KIND);
    }

    @Override
    public Symbol translate(Symbol adaptee) {
        assert adaptee instanceof EMAPortArraySymbol;
        return new PortArraySymbol2IODeclarationSymbol((EMAPortArraySymbol) adaptee);
    }
}
