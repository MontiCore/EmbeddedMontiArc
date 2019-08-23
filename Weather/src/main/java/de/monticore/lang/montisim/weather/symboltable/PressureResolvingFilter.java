/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class PressureResolvingFilter extends CommonResolvingFilter<PressureSymbol> {
  public PressureResolvingFilter() {
    super(PressureSymbol.KIND);
  }
}
