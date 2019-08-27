/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class SightResolvingFilter extends CommonResolvingFilter<SightSymbol> {
  public SightResolvingFilter() {
    super(SightSymbol.KIND);
  }
}
