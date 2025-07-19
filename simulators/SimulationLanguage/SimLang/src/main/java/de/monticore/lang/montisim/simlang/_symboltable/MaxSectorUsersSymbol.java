/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.util.types.AlternativeInput;

public class MaxSectorUsersSymbol extends CommonSymbol{

  public static final MaxSectorUsersKind KIND = MaxSectorUsersKind.INSTANCE;
  private AlternativeInput maxSectorUsers;

  public MaxSectorUsersSymbol(String name, AlternativeInput maxSectorUsers) {
    super(name, KIND);
    this.maxSectorUsers = maxSectorUsers;
  }

  public AlternativeInput getMaxSectorUsers() {
    return maxSectorUsers;
  }
}
