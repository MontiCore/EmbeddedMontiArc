/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.collector;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;

import java.util.List;

public interface ECCollector {
    /**
     * @return A list of all instructions scattered among different files.
     */
    List<ASTInstruction> collect();
}
