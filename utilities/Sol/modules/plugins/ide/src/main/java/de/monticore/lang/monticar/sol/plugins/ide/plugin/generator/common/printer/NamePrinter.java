/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer;

import de.monticore.symboltable.Symbol;

public interface NamePrinter extends IDEPrinter {
    String printMethodNameSuffix(Symbol symbol);
    String printQualifiedClassPrefix(Symbol symbol);
    String printQualifiedPath(Symbol symbol);
}
