/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class TimeResolvingFilter extends CommonResolvingFilter<TimeSymbol> {
  public TimeResolvingFilter() {
    super(TimeSymbol.KIND);
  }
}
