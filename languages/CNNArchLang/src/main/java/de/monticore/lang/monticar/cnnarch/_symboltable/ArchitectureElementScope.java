/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.Collection;
import java.util.Optional;

public class ArchitectureElementScope extends de.monticore.symboltable.CommonScope {

    public ArchitectureElementScope() {
        super(true);
    }

    public ArchitectureElementScope(Optional<MutableScope> enclosingScope) {
        super(enclosingScope, true);
    }

    @Override
    public void setResolvingFilters(Collection<ResolvingFilter<? extends Symbol>> resolvingFilters) {
        super.setResolvingFilters(resolvingFilters);
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
