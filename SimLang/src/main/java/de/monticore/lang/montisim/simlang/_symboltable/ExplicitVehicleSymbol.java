/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.util.types.ExplicitVehicle;

// extending ExplicitVehicleSymbolTOP breaks SimLangContainerTest
public class ExplicitVehicleSymbol extends CommonSymbol {

  public static final ExplicitVehicleKind KIND = ExplicitVehicleKind.INSTANCE;
  private ExplicitVehicle vehicles;

  public ExplicitVehicleSymbol(String name, ExplicitVehicle vehicles) {
    super(name, KIND);
    this.vehicles = vehicles;
  }

  @Deprecated
  public ExplicitVehicleSymbol(String name) {
    // this is needed so that the TOP class compiles
    super(name, KIND);
  }

  public ExplicitVehicle getVehicle() {
    return vehicles;
  }
}
