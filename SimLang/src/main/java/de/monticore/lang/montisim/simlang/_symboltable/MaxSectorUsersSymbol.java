package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class MaxSectorUsersSymbol extends CommonSymbol{

  public static final MaxSectorUsersSymbolKind KIND = MaxSectorUsersSymbolKind.INSTANCE;
  private ValueListRangeLambda maxSectorUsers;

  public MaxSectorUsersSymbol(String name, ValueListRangeLambda maxSectorUsers) {
    super(name, KIND);
    this.maxSectorUsers = maxSectorUsers;
  }

  public ValueListRangeLambda getMaxSectorUsers() {
    return maxSectorUsers;
  }
}
