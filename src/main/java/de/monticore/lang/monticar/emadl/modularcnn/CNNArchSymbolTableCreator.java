/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;

import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchSymbolTableCreatorTOP;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Deque;

public class CNNArchSymbolTableCreator extends CNNArchSymbolTableCreatorTOP {
    public CNNArchSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public CNNArchSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }


}
