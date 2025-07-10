/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.clustering.helpers;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;

import java.util.*;
import java.util.stream.Collectors;

public class ComponentHelper {
    public static List<EMAComponentInstanceSymbol> getSubcompsOrderedByName(EMAComponentInstanceSymbol componentInstanceSymbol){
        return componentInstanceSymbol.getSubComponents().stream()
                .sorted(Comparator.comparing(EMAComponentInstanceSymbol::getFullName))
                .collect(Collectors.toList());
    }

    public static Collection<EMAConnectorInstanceSymbol> getInnerConnectors(EMAComponentInstanceSymbol componentInstanceSymbol){
        String superCompName = componentInstanceSymbol.getFullName();
        return componentInstanceSymbol.getConnectorInstances().stream()
                //filter out all connectors to super component
                .filter(con -> !con.getSourcePort().getComponentInstance().getFullName().equals(superCompName)
                        && !con.getTargetPort().getComponentInstance().getFullName().equals(superCompName))
                .collect(Collectors.toList());
    }

    public static Map<String, Integer> getLabelsForSubcomps(List<EMAComponentInstanceSymbol> subcomps) {
        Map<String, Integer> componentIndecies = new HashMap<>();

        int[] i = {0};
        subcomps.forEach(sc -> componentIndecies.put(sc.getFullName(), i[0]++));
        return componentIndecies;
    }

    public static Map<Integer, String> getSubcompsLabels(List<EMAComponentInstanceSymbol> subcomps) {
        Map<Integer, String> componentIndecies = new HashMap<>();

        int[] i = {0};
        subcomps.forEach(sc -> componentIndecies.put(i[0]++, sc.getFullName()));
        return componentIndecies;
    }
}
