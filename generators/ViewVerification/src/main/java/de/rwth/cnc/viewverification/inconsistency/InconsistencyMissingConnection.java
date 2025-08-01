/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

public class InconsistencyMissingConnection extends Inconsistency {

  private String componentSource;
  private String componentTarget;
  private String portSource;
  private String portTarget;

  public InconsistencyMissingConnection(String componentSource, String componentTarget, String portSource, String portTarget) {
    this.componentSource = componentSource;
    this.componentTarget = componentTarget;
    this.portSource = portSource;
    this.portTarget = portTarget;
  }

  public String getComponentSource() {
    return componentSource;
  }

  public String getComponentTarget() {
    return componentTarget;
  }

  public String getPortSource() {
    return portSource;
  }

  public String getPortTarget() {
    return portTarget;
  }

}
