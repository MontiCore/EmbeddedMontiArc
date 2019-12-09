/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;

import java.util.List;

public interface ECSanitizer {
    /**
     * This method triggers sanitization of a list of instructions given as parameter. Sanitization in this context
     * means resolving discrepancies which can be fixed without user involvement (e.g duplicate package installation).
     * @param instructions A list of instructions to be sanitized.
     * @return The sanitized list of instructions.
     */
    List<ASTInstruction> sanitize(List<ASTInstruction> instructions);
}
