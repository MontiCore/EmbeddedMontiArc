/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ddf.sanitizer;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;

import java.util.List;

public interface DDFSanitizerPhase {
    void sanitize(List<ASTInstruction> instructions);
}
