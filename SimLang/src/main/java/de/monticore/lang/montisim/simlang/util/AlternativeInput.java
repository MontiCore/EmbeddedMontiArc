package de.monticore.lang.montisim.simlang.util;

//Feel free to find a better name <3

import de.monticore.lang.montisim.weather.cocos.NumberUnit;
import jline.internal.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AlternativeInput {
  private Optional<NumberUnit> nUnit = Optional.empty();
  private Optional<ArrayList<NumberUnit>> list = Optional.empty();
  private Optional<Range> range = Optional.empty();

  public AlternativeInput(NumberUnit nu) {
    this.nUnit = Optional.of(nu);
  }
  public AlternativeInput(ArrayList list) {
    this.list = Optional.of(list);
  }
  public AlternativeInput(Range range) {
    this.range = Optional.of(range);
  }

  public Optional<NumberUnit> getNUnit() {
    return this.nUnit;
  }

  public Optional<ArrayList<NumberUnit>> getList() {
    return this.list;
  }

  public Optional<Range> getRange() {
    return this.range;
  }


}
