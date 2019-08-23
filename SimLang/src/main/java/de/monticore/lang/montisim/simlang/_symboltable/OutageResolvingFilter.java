/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class OutageResolvingFilter extends CommonResolvingFilter<OutageSymbol> {
  public OutageResolvingFilter() {
    super(OutageSymbol.KIND);
  }
}
