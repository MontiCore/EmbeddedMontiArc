/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.monticar.semantics.construct.InstanceCreator;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;

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
                    .filter(c -> c.getTarget().startsWith(component.get().getName()) || c.getSource().startsWith(component.get().getName()))
                    .forEachOrdered(c -> enclosingScope.remove(c));
        }
    }

    public static EMAComponentInstanceSymbol resolveInstance(GlobalScope scope, String fullQualifiedName) {
        return scope.<EMAComponentInstanceSymbol>resolve(
                fullQualifiedName, EMAComponentInstanceSymbol.KIND).orElse(null);
    }

    public static EMAComponentInstanceSymbol resolveInstanceTo(GlobalScope scope, String fullQualifiedName, EMAComponentInstanceSymbol parent) {
        EMAComponentInstanceSymbol instance = resolveInstance(scope, fullQualifiedName);
        instance.setPackageName(parent.getFullName());
        instance.setFullName(Joiners.DOT.join(parent.getFullName(), NameHelper.getName(fullQualifiedName)));
        instance.getEnclosingScope().getAsMutableScope().remove(instance);
        parent.getSpannedScope().getAsMutableScope().add(instance);
        return instance;
    }
}
