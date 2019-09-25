/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;

public class CheckElementInputs extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        for (CompositeElementSymbol stream : architecture.getStreams()) {
            stream.checkInput();
        }
    }
}
