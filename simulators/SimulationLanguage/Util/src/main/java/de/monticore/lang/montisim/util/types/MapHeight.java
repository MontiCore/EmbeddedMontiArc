/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

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

  public Optional<SimLangEnums.SimulationHeightModes> getHeightMode() {
    return heightMode;
  }

  public Optional<String> getCustomHeight() {
    return customHeight;
  }
}
