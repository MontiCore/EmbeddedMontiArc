/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class HumidityResolvingFilter extends CommonResolvingFilter<HumiditySymbol> {
  public HumidityResolvingFilter() {
    super(HumiditySymbol.KIND);
  }
}
