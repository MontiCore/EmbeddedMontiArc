/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class WeatherResolvingFilter extends CommonResolvingFilter<WeatherSymbol> {
  public WeatherResolvingFilter() {
    super(WeatherSymbol.KIND);
  }
}
