/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.setup;

import de.monticore.lang.monticar.setup.action.Action;
import java.util.List;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

/**
 * This class executes actions defined by setup configurations.
 *
 * @see Configuration
 */
@Component
public class AutoSetup {

  private List<Configuration> configs;

  @Autowired
  public AutoSetup(List<Configuration> configs) {
    this.configs = configs;
  }

  /**
   * Executes every action defined by the {@link Configuration}s in the
   * intended order.
   */
  public void setup() {
    for (Configuration config : configs) {
      for (Action action : config.getActions()) {
        action.execute();
      }
    }
  }
}
