/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.executionOrder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.Find;

import java.util.*;

import static de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper.isAtomic;

public class SList {

    public static List<List<SListEntry>> sListParallel(EMAComponentInstanceSymbol rootComponent) {
        List<List<SListEntry>> sList = new LinkedList<>();

        // This List is sorted for hierarchy
        List<EMAComponentInstanceSymbol> components = Find.allAtomicOrNVComponents(rootComponent);
        int maxIndex = components.stream()
                .filter(p -> !p.getOrderOutput().isEmpty() && !p.getOrderUpdate().equals(Integer.MAX_VALUE))
                .map(p -> Integer.max(Collections.max(p.getOrderOutput()), p.getOrderUpdate()))
                .max(Integer::compareTo).orElse(0);

        for (int currentIndex = 1; currentIndex <= maxIndex; currentIndex++) {
            ListIterator<EMAComponentInstanceSymbol> iterator = components.listIterator();
            List<SListEntry> currentList = new LinkedList<>();
            sList.add(currentList);
            addAllWithCurrentIndex(currentIndex, iterator, currentList);
        }

        return sList;
    }

    public static List<SListEntry> sListSerial(EMAComponentInstanceSymbol rootComponent) {
        List<SListEntry> sList = new LinkedList<>();

        // This List is sorted for hierarchy
        List<EMAComponentInstanceSymbol> components = Find.allAtomicOrNVComponents(rootComponent);
        int maxIndex = components.stream()
                .filter(p -> !p.getOrderOutput().isEmpty() && !p.getOrderUpdate().equals(Integer.MAX_VALUE))
                .map(p -> Integer.max(Collections.max(p.getOrderOutput()), p.getOrderUpdate()))
                .max(Integer::compareTo).orElse(0);

        for (int currentIndex = 1; currentIndex <= maxIndex; currentIndex++) {
            ListIterator<EMAComponentInstanceSymbol> iterator = components.listIterator();
            addAllWithCurrentIndex(currentIndex, iterator, sList);
        }

        return sList;
    }

    private static void addAllWithCurrentIndex(int currentIndex, ListIterator<EMAComponentInstanceSymbol> iterator, List<SListEntry> currentList) {
        while (iterator.hasNext()) {
            EMAComponentInstanceSymbol component = iterator.next();
            if (component.getOrderOutput().contains(currentIndex)) {
                if (component.getOrderUpdate() == currentIndex + 1) {
                    currentList.add(new SListEntry(component, Call.EXECUTE));
                    iterator.remove();
                } else {
                    currentList.add(new SListEntry(component, Call.OUTPUT));
                }
            } else if (component.getOrderUpdate() == currentIndex) {
                currentList.add(new SListEntry(component, Call.UPDATE));
                iterator.remove();
            }
        }
    }

    private static boolean isContainedIn(EMAComponentInstanceSymbol component, EMAComponentInstanceSymbol subSystem) {
        if (isAtomic(subSystem)) return false;
        return component.getFullName().startsWith(subSystem.getFullName());
    }

}
