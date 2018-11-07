package de.monticore.lang.monticar.generator.middleware.helpers;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

import java.util.*;
import java.util.stream.Collectors;

public class ComponentHelper {
    public static List<ExpandedComponentInstanceSymbol> getSubcompsOrderedByName(ExpandedComponentInstanceSymbol componentInstanceSymbol){
        return componentInstanceSymbol.getSubComponents().stream()
                .sorted(Comparator.comparing(ExpandedComponentInstanceSymbol::getFullName))
                .collect(Collectors.toList());
    }

    public static Collection<ConnectorSymbol> getInnerConnectors(ExpandedComponentInstanceSymbol componentInstanceSymbol){
        String superCompName = componentInstanceSymbol.getFullName();
        return componentInstanceSymbol.getConnectors().stream()
                //filter out all connectors to super component
                .filter(con -> !con.getSourcePort().getComponentInstance().get().getFullName().equals(superCompName)
                        && !con.getTargetPort().getComponentInstance().get().getFullName().equals(superCompName))
                .collect(Collectors.toList());
    }

    public static Map<String, Integer> getLabelsForSubcomps(List<ExpandedComponentInstanceSymbol> subcomps) {
        Map<String, Integer> componentIndecies = new HashMap<>();

        int[] i = {0};
        subcomps.forEach(sc -> componentIndecies.put(sc.getFullName(), i[0]++));
        return componentIndecies;
    }
}
