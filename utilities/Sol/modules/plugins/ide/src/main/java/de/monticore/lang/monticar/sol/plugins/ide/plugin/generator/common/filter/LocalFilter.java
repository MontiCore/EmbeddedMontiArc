/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.filter;

import de.monticore.symboltable.Symbol;

import java.util.Set;

public interface LocalFilter {
    <S extends Symbol> Set<S> filter(Set<S> configurations);
}
