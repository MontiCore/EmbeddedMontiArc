/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;

import java.util.Optional;

public class LayerDeclarationScope extends de.monticore.symboltable.CommonScope {

    public LayerDeclarationScope() {
        super(true);
    }

    public LayerDeclarationScope(Optional<MutableScope> enclosingScope) {
        super(enclosingScope, true);
    }

    @Override
    public void add(Symbol symbol) {
        super.add(symbol);
        if (symbol instanceof ArchitectureElementSymbol){
            ArchitectureElementScope subScope = ((ArchitectureElementSymbol) symbol).getSpannedScope();
            addSubScope(subScope);
            subScope.setResolvingFilters(getResolvingFilters());
        }
    }
}
