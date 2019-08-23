/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class AreaResolvingFilter extends CommonResolvingFilter<AreaSymbol> {
  public AreaResolvingFilter() {
    super(AreaSymbol.KIND);
  }
}
