package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class PedestrianDensitySymbol extends CommonSymbol{

  public static final PedestrianDensitySymbolKind KIND = PedestrianDensitySymbolKind.INSTANCE;
  private ValueListRangeLambda pedestrianDensity;

  public PedestrianDensitySymbol(String name, ValueListRangeLambda pedestrianDensity) {
    super(name, KIND);
    this.pedestrianDensity = pedestrianDensity;
  }

  public ValueListRangeLambda getPedestrianDensity() {
    return pedestrianDensity;
  }
}
