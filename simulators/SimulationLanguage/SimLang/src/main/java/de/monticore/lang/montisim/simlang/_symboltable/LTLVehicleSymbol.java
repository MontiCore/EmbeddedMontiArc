/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.LTLVehicle;
import de.monticore.symboltable.CommonSymbol;

public class LTLVehicleSymbol extends CommonSymbol{

  public static final LTLVehicleKind KIND = LTLVehicleKind.INSTANCE;
  private LTLVehicle vehicle;

  public LTLVehicleSymbol(String name, LTLVehicle vehicle) {
    super(name, KIND);
    this.vehicle = vehicle;
  }

  public LTLVehicle getVehicle() {
    return vehicle;
  }
}
