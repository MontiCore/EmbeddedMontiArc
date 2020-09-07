/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;

import java.util.Collection;
import java.util.Optional;

public class SymbolTableHelper {

    public static void removeComponent(GlobalScope scope, String s) {
        Optional<EMAComponentInstanceSymbol> component = scope.resolve(s, EMAComponentInstanceSymbol.KIND);
        if (component.isPresent())
            component.get().getEnclosingScope().getAsMutableScope().remove(component.get());
    }

    public static void removeComponentWithConnectors(GlobalScope scope, String s) {
        Optional<EMAComponentInstanceSymbol> component = scope.resolve(s, EMAComponentInstanceSymbol.KIND);
        if (component.isPresent()) {
            MutableScope enclosingScope = component.get().getEnclosingScope().getAsMutableScope();
            enclosingScope.remove(component.get());
            Collection<EMAConnectorInstanceSymbol> connectors = enclosingScope.resolveLocally(EMAConnectorInstanceSymbol.KIND);
            connectors.stream()
                    .filter(c -> c.getTarget().equals(component.get().getName()))
                    .forEachOrdered(c -> enclosingScope.remove(c));
        }
    }
}
