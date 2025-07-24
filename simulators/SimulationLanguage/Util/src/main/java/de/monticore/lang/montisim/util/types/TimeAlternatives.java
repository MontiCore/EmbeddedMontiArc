/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.ArrayList;
import java.util.Optional;

public class TimeAlternatives {
  private Optional<Time> single = Optional.empty();
  private Optional<ArrayList> list = Optional.empty();

  public TimeAlternatives(Time single) {
    this.single = Optional.of(single);
  }
  public TimeAlternatives(ArrayList<Time> list) {
    this.list = Optional.of(list);
  }

  public Optional<Time> getSingle() {
    return single;
  }

  public Optional<ArrayList> getList() {
    return list;
  }
}
