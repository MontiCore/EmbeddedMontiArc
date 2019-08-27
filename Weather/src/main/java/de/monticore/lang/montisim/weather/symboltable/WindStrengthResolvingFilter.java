/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class WindStrengthResolvingFilter extends CommonResolvingFilter<WindStrengthSymbol> {
  public WindStrengthResolvingFilter() {
    super(WindStrengthSymbol.KIND);
  }
}
