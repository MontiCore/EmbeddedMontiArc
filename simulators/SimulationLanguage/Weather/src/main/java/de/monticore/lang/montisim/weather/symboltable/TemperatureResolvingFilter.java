/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class TemperatureResolvingFilter extends CommonResolvingFilter<TemperatureSymbol> {
  public TemperatureResolvingFilter() {
    super(TemperatureSymbol.KIND);
  }
}
