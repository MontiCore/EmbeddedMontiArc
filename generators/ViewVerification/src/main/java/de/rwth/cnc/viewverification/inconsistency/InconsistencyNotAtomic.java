/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

import java.util.List;

public class InconsistencyNotAtomic extends Inconsistency {
  private String componentName;
  private List<String> subcomponentNames;

  public InconsistencyNotAtomic(String componentName, List<String> subcomponentNames) {
    this.componentName = componentName;
    this.subcomponentNames = subcomponentNames;
  }

  public String getComponentName() {
    return componentName;
  }

  public List<String> getSubcomponentNames() {
    return subcomponentNames;
  }
}
