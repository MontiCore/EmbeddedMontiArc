/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.Pedestrian;
import de.monticore.symboltable.CommonSymbol;

public class PedestrianSymbol extends CommonSymbol{

  public static final PedestrianKind KIND = PedestrianKind.INSTANCE;
  private Pedestrian pedestrian;

  public PedestrianSymbol(String name, Pedestrian pedestrian) {
    super(name, KIND);
    this.pedestrian = pedestrian;
  }

  public Pedestrian getPedestrian() {
    return pedestrian;
  }
}
