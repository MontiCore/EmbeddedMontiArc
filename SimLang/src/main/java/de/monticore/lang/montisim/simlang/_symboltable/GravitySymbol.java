/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class GravitySymbol extends CommonSymbol{

  public static final GravityKind KIND = GravityKind.INSTANCE;
  private AlternativeInput gravity;

  public GravitySymbol(String name, AlternativeInput gravity) {
    super(name, KIND);
    this.gravity = gravity;
  }

  public AlternativeInput getGravity() {
    return gravity;
  }
}
