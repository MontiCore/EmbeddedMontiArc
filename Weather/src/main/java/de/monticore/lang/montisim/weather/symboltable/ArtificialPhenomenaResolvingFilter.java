package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class ArtificialPhenomenaResolvingFilter extends CommonResolvingFilter<ArtificialPhenomenaSymbol> {
  public ArtificialPhenomenaResolvingFilter() {
    super(ArtificialPhenomenaSymbol.KIND);
  }
}