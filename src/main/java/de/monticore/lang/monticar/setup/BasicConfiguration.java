/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup;

import de.monticore.lang.monticar.setup.action.Action;
import java.util.Arrays;
import java.util.List;

public class BasicConfiguration implements Configuration {

  private final Action[] actions;

  public BasicConfiguration(Action... actions) {
    this.actions = actions;
  }

  public List<Action> getActions() {
    return Arrays.asList(actions);
  }
}
