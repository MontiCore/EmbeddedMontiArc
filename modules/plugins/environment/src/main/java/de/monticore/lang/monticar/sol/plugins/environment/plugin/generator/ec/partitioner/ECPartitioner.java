/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.partitioner;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;

import java.util.List;

public interface ECPartitioner {
    /**
     * This method triggers the partitioning of a list given as parameter. The resulting partition will collect all
     * consecutive instructions of the same type. For example {ARG ARG ARG INSTALL INSTALL ENV} will become
     * {{ARG, ARG, ARG}, {INSTALL, INSTALL}, {ENV}}.
     * @param instructions A list of instructions which should be partitioned.
     * @return The partitioned list of instructions.
     */
    List<List<ASTInstruction>> partition(List<ASTInstruction> instructions);
}
