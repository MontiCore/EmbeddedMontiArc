/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator.serializer;

import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;

public interface OptionsSerializer {
    String serialize(SymbolWithOptions symbol);
}
