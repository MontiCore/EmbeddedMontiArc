/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import java.util.Optional;

public class Sight {
  private Optional<AlternativeInput> sight = Optional.empty();

  public Sight () {}
  public Sight(AlternativeInput sight) {
    if(sight != null) {
      this.sight = Optional.of(sight);
    }
  }

  public Optional<AlternativeInput> getSight() {
    return sight;
  }

  public boolean isUnlimited() {
    return !this.sight.isPresent();
  }
}
