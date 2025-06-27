/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceBuilder;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbolCreator;

public class ModifiedEMAComponentInstanceSymbolCreator extends EMADynamicComponentInstanceSymbolCreator {
    @Override
    protected EMADynamicComponentInstanceBuilder getNewBuilder() {
        return new ModifiedEMAComponentInstanceBuilder();
    }
}
