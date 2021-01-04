/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.isAtomic;
import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.isNonVirtual;

public class Find {

    public static LinkedList<EMAComponentInstanceSymbol> allComponents(EMAComponentInstanceSymbol component) {
        LinkedList<EMAComponentInstanceSymbol> result = new LinkedList<>();
        result.add(component);
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(allComponents(subComponent));
        return result;
    }

    public static LinkedList<EMAComponentInstanceSymbol> allAtomicOrNVComponents(EMAComponentInstanceSymbol component) {
        return allAtomicOrNVComponents(component, true);
    }

    private static LinkedList<EMAComponentInstanceSymbol> allAtomicOrNVComponents(EMAComponentInstanceSymbol component,
                                                                           boolean firstCall) {
        if (isAtomic(component) || isNonVirtual(component) && !firstCall)
            return new LinkedList(Collections.singletonList(component));

        LinkedList<EMAComponentInstanceSymbol> result = new LinkedList<>();
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(allAtomicOrNVComponents(subComponent, false));
        return result;
    }

    public static LinkedList<EMAComponentInstanceSymbol> allSubSystems(EMAComponentInstanceSymbol component) {
        return allSubSystems(component, true);
    }

    private static LinkedList<EMAComponentInstanceSymbol> allSubSystems(EMAComponentInstanceSymbol component,
                                                                  boolean firstCall) {
        if (isAtomic(component))
            return new LinkedList<>();

        LinkedList<EMAComponentInstanceSymbol> result = new LinkedList<>();
        if (isNonVirtual(component) && !firstCall)
            result.add(component);
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(allSubSystems(subComponent, false));
        return result;
    }
}
