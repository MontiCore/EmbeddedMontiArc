/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.translator;

import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;

import java.util.List;

public interface ECTranslator {
    /**
     * This method invokes pretty-printing of a partitioned list of instructions.
     * @param partitions The partitions to be pretty-printed.
     * @param engine The GeneratorEngine which will be used to execute the templates.
     * @return The pretty-printed list of instructions.
     */
    String translate(List<List<ASTInstruction>> partitions, GeneratorEngine engine);
}
