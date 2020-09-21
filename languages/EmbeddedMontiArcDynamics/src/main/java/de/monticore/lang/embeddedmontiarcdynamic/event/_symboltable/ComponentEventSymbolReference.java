/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.Scope;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;

public class ComponentEventSymbolReference extends ComponentEventSymbolReferenceTOP  {
    public ComponentEventSymbolReference(String name, Scope enclosingScopeOfReference) {
        super(name, enclosingScopeOfReference);
    }

    public EventExpressionSymbol getCondition(){
        return this.getReferencedSymbol().getCondition();
    }

    @Override
    public List<MCTypeSymbol> getFormalTypeParameters() {
        return this.getReferencedSymbol().getFormalTypeParameters();
//        final Collection<MCTypeSymbol> resolvedTypes = this.getSpannedScope().resolveLocally(MCTypeSymbol.KIND);
//        return resolvedTypes.stream().filter(MCTypeSymbol::isFormalTypeParameter)
//                .collect(Collectors.toList());
    }

    public List<EMAVariable> getParameters() {
        return this.getReferencedSymbol().getParameters();
    }


}
