package de.monticore.lang.montisim.simlang.util;

import java.util.Optional;

public class MapHeight {
  private Optional<String> customHeight = Optional.empty();
  private Optional<SimLangEnums.SimulationHeightModes> heightMode = Optional.empty();

  public MapHeight(Optional opt) {
    this.customHeight = opt;
  }
  public MapHeight(SimLangEnums.SimulationHeightModes mode) {
    this.heightMode = Optional.of(mode);
  }
}
