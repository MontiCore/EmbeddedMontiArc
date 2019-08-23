/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.util.types.ExplicitVehicle;

public class ExplicitVehicleSymbol extends CommonSymbol{

  public static final ExplicitVehicleKind KIND = ExplicitVehicleKind.INSTANCE;
  private ExplicitVehicle vehicles;

  public ExplicitVehicleSymbol(String name, ExplicitVehicle vehicles) {
    super(name, KIND);
    this.vehicles = vehicles;
  }

  public ExplicitVehicle getVehicle() {
    return vehicles;
  }
}
