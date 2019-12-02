/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;

public class CheckArchitecture{

    public static void check(EMAComponentInstanceSymbol instance) {
        ArchitectureSymbol architecture = instance.getSpannedScope().
                <ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        CNNArchCocos.checkAll(architecture);
    }
}
