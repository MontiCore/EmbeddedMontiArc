package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class PrecipitationTypeResolvingFilter extends CommonResolvingFilter<PrecipitationTypeSymbol> {
  public PrecipitationTypeResolvingFilter() {
    super(PrecipitationTypeSymbol.KIND);
  }
}