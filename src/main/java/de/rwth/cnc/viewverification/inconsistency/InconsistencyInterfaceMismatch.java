/**
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
