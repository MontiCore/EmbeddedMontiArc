/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 */
public class InstancingRegister {
    public static List<InstanceInformation> instanceInformation = new ArrayList();
    public static String mainComponent = "defaultComponent";
    public static String mainInstantiation = "defaultInstantiation";

    public static void addInstanceInformation(InstanceInformation i) {
        instanceInformation.add(i);
        Log.info(i.toString(), "Added InstanceInformation");
    }

    public static Optional<InstanceInformation> getInstanceInformation(String name) {
        for (InstanceInformation i : instanceInformation) {
            if (i.getCompName().equals(name))
                return Optional.of(i);
        }
        return Optional.empty();
    }

    public static void reset() {
        instanceInformation.clear();
        mainComponent = "defaultComponent";
        mainInstantiation = "defaultInstantiation";
    }
}
