/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.symboltable.Symbol;

public interface ImportPrinter extends NamePrinter {
    String printImport(SolPackage rootPackage, Symbol symbol, String extension, String relativePath);
}
