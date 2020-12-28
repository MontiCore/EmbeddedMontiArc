/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.isAtomic;
import static de.monticore.lang.monticar.semantics.loops.detection.ConnectionHelper.isNonVirtual;

public class Find {

    public static List<EMAComponentInstanceSymbol> allAtomicOrNVComponents(EMAComponentInstanceSymbol component) {
        return allAtomicOrNVComponents(component, true);
    }

    private static List<EMAComponentInstanceSymbol> allAtomicOrNVComponents(EMAComponentInstanceSymbol component,
                                                                           boolean firstCall) {
        if (isAtomic(component) || isNonVirtual(component) && !firstCall)
            return Collections.singletonList(component);

        List<EMAComponentInstanceSymbol> result = new LinkedList<>();
        for (EMAComponentInstanceSymbol subComponent : component.getSubComponents())
            result.addAll(allAtomicOrNVComponents(subComponent, false));
        return result;
    }
}
