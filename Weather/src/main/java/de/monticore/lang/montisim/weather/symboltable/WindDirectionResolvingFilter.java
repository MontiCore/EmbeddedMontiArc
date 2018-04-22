package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class WindDirectionResolvingFilter extends CommonResolvingFilter<WindDirectionSymbol> {
  public WindDirectionResolvingFilter() {
    super(WindDirectionSymbol.KIND);
  }
}