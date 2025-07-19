/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

public class InconsistencyMissingComponent extends Inconsistency {

  private String componentName;

  public InconsistencyMissingComponent(String componentName) {
    this.componentName = componentName;
  }

  public String getComponentName() {
    return componentName;
  }
}
