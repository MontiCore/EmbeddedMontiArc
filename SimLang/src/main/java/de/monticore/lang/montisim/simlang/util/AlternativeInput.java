package de.monticore.lang.montisim.simlang.util;

//Feel free to find a better name <3

import java.util.ArrayList;
import java.util.Optional;

public class AlternativeInput {
  private Optional<NumberUnit> nUnit = Optional.empty();
  private Optional<ArrayList> list = Optional.empty();
  private Optional<Range> range = Optional.empty();
  private Optional<Lambda> lambda = Optional.empty();

  public AlternativeInput(Optional opt) {
    if(opt.get() instanceof NumberUnit) {
      this.nUnit = opt;
    }
    else if(opt.get() instanceof ArrayList) {
      this.list = opt;
    }
    else if(opt.get() instanceof Range) {
      this.range = opt;
    }
    else{
      this.lambda = opt;
    }
  }

  public Optional<NumberUnit> getNUnit() {
    return this.nUnit;
  }

  public Optional<ArrayList> getList() {
    return this.list;
  }

  public Optional<Range> getRange() {
    return this.range;
  }

  public Optional<Lambda> getLambda() {
    return this.lambda;
  }

}
