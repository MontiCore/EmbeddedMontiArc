package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class PedestrianDensityResolvingFilter extends CommonResolvingFilter<PedestrianDensitySymbol> {
  public PedestrianDensityResolvingFilter() {
    super(PedestrianDensitySymbol.KIND);
  }
}