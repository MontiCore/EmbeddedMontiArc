/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class TimeoutResolvingFilter extends CommonResolvingFilter<TimeoutSymbol> {
  public TimeoutResolvingFilter() {
    super(TimeoutSymbol.KIND);
  }
}
