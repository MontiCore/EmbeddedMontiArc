/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

public class EMAPropertiesHelper {

    public static boolean isAtomic(EMAComponentInstanceSymbol componentInstance) {
        return componentInstance.getSubComponents().isEmpty() && componentInstance.getConnectorInstances().isEmpty();
    }

    public static boolean isNonVirtual(EMAComponentInstanceSymbol componentInstanceSymbol) {
        return componentInstanceSymbol.isNonVirtual() || !componentInstanceSymbol.getParent().isPresent();
    }
}
