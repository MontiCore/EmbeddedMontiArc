/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emscripten;

import static de.monticore.lang.monticar.contract.StringPrecondition.requiresNotBlank;

/**
 * This class represents a command line option with boolean values of the type
 * {@code option=1} or {@code option=0}.
 */
public class Option {

  private final String option;
  private final boolean enabled;

  /**
   * Initializes a new option with the specified option title and boolean value.
   * {@code true} represents 1, {@code false} represents 0.
   *
   * @param option option title
   * @param enabled option value
   */
  public Option(String option, boolean enabled) {
    this.option = requiresNotBlank(option);
    this.enabled = enabled;
  }

  public String getOption() {
    return option;
  }

  public boolean isEnabled() {
    return enabled;
  }

  @Override
  public String toString() {
    return option + "=" + (enabled ? "1" : "0");
  }
}
