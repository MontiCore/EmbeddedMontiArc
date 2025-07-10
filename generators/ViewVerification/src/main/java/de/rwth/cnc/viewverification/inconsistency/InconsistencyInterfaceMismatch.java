/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

public class InconsistencyInterfaceMismatch extends Inconsistency {

  private String componentName;
  private String portName;
  private String portType;
  private Boolean incoming;
  private InconsistencyInterfaceMismatchKind mismatchKind;

  public InconsistencyInterfaceMismatch(String componentName, String portName, String portType, Boolean incoming, InconsistencyInterfaceMismatchKind mismatchKind) {
    this.componentName = componentName;
    this.portName = portName;
    this.portType = portType;
    this.incoming = incoming;
    this.mismatchKind = mismatchKind;
  }

  public String getComponentName() {
    return componentName;
  }

  public String getPortName() {
    return portName;
  }

  public String getPortType() {
    return portType;
  }

  public Boolean getIncoming() {
    return incoming;
  }

  public InconsistencyInterfaceMismatchKind getMismatchKind() {
    return mismatchKind;
  }

}
