/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class WeatherPhenomenaResolvingFilter extends CommonResolvingFilter<WeatherPhenomenaSymbol> {
  public WeatherPhenomenaResolvingFilter() {
    super(WeatherPhenomenaSymbol.KIND);
  }
}
