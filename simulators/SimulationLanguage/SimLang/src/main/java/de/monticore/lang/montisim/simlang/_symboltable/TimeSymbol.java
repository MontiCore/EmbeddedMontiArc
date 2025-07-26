/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.util.types.Time;

import java.util.ArrayList;

public class TimeSymbol extends CommonSymbol{

  public static final TimeKind KIND = TimeKind.INSTANCE;
  private ArrayList<Time> time;

  public TimeSymbol(String name, ArrayList time) {
    super(name, KIND);
    this.time = time;
  }

  public ArrayList<Time> getTime() {
    return time;
  }
}
