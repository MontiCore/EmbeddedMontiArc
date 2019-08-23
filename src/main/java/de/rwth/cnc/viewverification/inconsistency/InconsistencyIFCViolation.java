/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

import de.rwth.cnc.model.Direction;
import de.rwth.cnc.model.Port;

public class InconsistencyIFCViolation extends Inconsistency {

  private String componentName;
  private String portName;
  private String portType;
  private Direction portDirection;

  public InconsistencyIFCViolation(String componentName, String portName, String portType, Direction portDirection) {
    this.componentName = componentName;
    this.portName = portName;
    this.portType = portType;
    this.portDirection = portDirection;
  }

  public InconsistencyIFCViolation(String componentName, Port port) {
    this(componentName, port.getName(), port.getType(), port.getDirection());
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

  public Direction getPortDirection() {
    return portDirection;
  }
}
