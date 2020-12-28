/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.executionOrder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.Find;

import java.util.*;

public class SList {

    public static List<SListEntry> sList(EMAComponentInstanceSymbol rootComponent) {
        List<SListEntry> result = new LinkedList<>();

        Set<EMAComponentInstanceSymbol> alreadyUpdated = new HashSet<>();

        // This List is sorted for hierarchy
        List<EMAComponentInstanceSymbol> components = Find.allAtomicOrNVComponents(rootComponent);
        int maxIndex = components.stream()
                .map(p -> Integer.max(Collections.max(p.getOrderOutput()), p.getOrderUpdate()))
                .mapToInt(v -> v).max().orElse(0);

        for (int currentIndex = 1; currentIndex <= maxIndex; currentIndex++) {
            for (EMAComponentInstanceSymbol component : components) {
                if (alreadyUpdated.contains(component)) continue;
                if (component.getOrderOutput().contains(currentIndex)) {
                    if (component.getOrderUpdate() == currentIndex + 1) {
                        result.add(new SListEntry(component, Call.EXECUTE));
                        alreadyUpdated.add(component);
                    } else {
                        result.add(new SListEntry(component, Call.OUTPUT));
                    }
                } else if (component.getOrderUpdate() == currentIndex) {
                    result.add(new SListEntry(component, Call.UPDATE));
                    alreadyUpdated.add(component);
                }
            }
        }

        return result;
    }

}
