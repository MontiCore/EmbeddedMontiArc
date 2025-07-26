/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.PathedVehicle;
import de.monticore.symboltable.CommonSymbol;

public class PathedVehicleSymbol extends CommonSymbol{

  public static final PathedVehicleKind KIND = PathedVehicleKind.INSTANCE;
  private PathedVehicle vehicle;

  public PathedVehicleSymbol(String name, PathedVehicle vehicle) {
    super(name, KIND);
    this.vehicle = vehicle;
  }

  public PathedVehicle getVehicle() {
    return vehicle;
  }
}
