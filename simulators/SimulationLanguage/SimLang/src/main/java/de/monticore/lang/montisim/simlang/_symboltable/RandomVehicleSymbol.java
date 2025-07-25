/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.RandomVehicle;
import de.monticore.symboltable.CommonSymbol;

public class RandomVehicleSymbol extends CommonSymbol{

  public static final RandomVehicleKind KIND = RandomVehicleKind.INSTANCE;
  private RandomVehicle vehicle;

  public RandomVehicleSymbol(String name, RandomVehicle vehicle) {
    super(name, KIND);
    this.vehicle = vehicle;
  }

  public RandomVehicle getVehicle() {
    return vehicle;
  }
}
