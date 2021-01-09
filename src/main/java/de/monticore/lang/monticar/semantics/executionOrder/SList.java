/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.executionOrder;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.Find;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.isAtomic;

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

    public static List<SListEntry> sListAtomic(EMAComponentInstanceSymbol rootComponent) {
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

    public static Map<EMAComponentInstanceSymbol, List<SListEntry>> sListSubSystem(EMAComponentInstanceSymbol rootComponent) {
        return sListsSubSystem(rootComponent, sListAtomic(rootComponent));
    }

    private static Map<EMAComponentInstanceSymbol, List<SListEntry>> sListsSubSystem(EMAComponentInstanceSymbol subSystem,
                                                                                     List<SListEntry> sListAtomic) {
        Map<EMAComponentInstanceSymbol, List<SListEntry>> sLists = new HashMap<>();
        if (isAtomic(subSystem)) return sLists;

        sLists.put(subSystem, sListSubSystem(subSystem, sListAtomic));

        for (EMAComponentInstanceSymbol subComponent : subSystem.getSubComponents()) {
            if (!isAtomic(subComponent))
                sLists.putAll(sListsSubSystem(subComponent, sListAtomic));
        }

        return sLists;
    }

    public static List<SListEntry> sListSubSystem(EMAComponentInstanceSymbol subsystem, List<SListEntry> sListAtomic) {
        List<SListEntry> sList = new LinkedList<>();
        Collection<EMAComponentInstanceSymbol> subComponents = subsystem.getSubComponents();

        boolean began = false;
        boolean finished = false;
        for (SListEntry sListEntry : sListAtomic) {
            if (isContainedIn(sListEntry.getComponent(), subsystem)) {
                if (finished)
                    Log.error("TODO Not supported, virtual subsystem creates artificial loop: " + subsystem.getFullName());
                began = true;
                if (subComponents.contains(sListEntry.getComponent()))
                    sList.add(sListEntry);
                else {
                    EMAComponentInstanceSymbol subComponent = subComponents.stream()
                            .filter(s -> isContainedIn(sListEntry.getComponent(), s))
                            .findFirst().orElse(null);
                    sList.add(new SListEntry(subComponent, sListEntry.getCall()));
                }
            } else if (began) finished = true;
        }

        return sList;
    }

    private static boolean isContainedIn(EMAComponentInstanceSymbol component, EMAComponentInstanceSymbol subSystem) {
        if (isAtomic(subSystem)) return false;
        return component.getFullName().startsWith(subSystem.getFullName());
    }

}
