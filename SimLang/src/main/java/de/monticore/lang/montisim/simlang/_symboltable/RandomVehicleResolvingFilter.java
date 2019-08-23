/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class RandomVehicleResolvingFilter extends CommonResolvingFilter<RandomVehicleSymbol> {
  public RandomVehicleResolvingFilter() {
    super(RandomVehicleSymbol.KIND);
  }
}
