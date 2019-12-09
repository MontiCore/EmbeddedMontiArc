/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.path;

import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;

import java.nio.file.Path;

public interface PathResolver {
    Path resolve(CommonLiterals origin, String path);
    Path resolve(CommonLiterals origin, Path path);
}
