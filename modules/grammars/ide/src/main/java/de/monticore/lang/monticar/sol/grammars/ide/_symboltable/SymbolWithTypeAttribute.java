/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.Optional;

public interface SymbolWithTypeAttribute extends Symbol {
    void setLabel(String label);
    String getLabel();

    void setIcon(String icon);
    Optional<String> getIcon();

    void setCategory(String category);
    Optional<String> getCategory();
}
