/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.order;

import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.TaskSymbol;

import java.util.List;

public interface OrderService {
    List<List<TaskSymbol>> orderTasks(ConfigurationTypeSymbol configuration);
    List<List<ConfigurationSymbol>> orderConfigurations(List<ConfigurationSymbol> configurations);
}
