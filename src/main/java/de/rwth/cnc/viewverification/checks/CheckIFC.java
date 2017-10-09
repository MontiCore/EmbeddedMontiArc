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
package de.rwth.cnc.viewverification.checks;

import de.rwth.cnc.model.CnCArchitecture;
import de.rwth.cnc.model.CnCView;
import de.rwth.cnc.model.Component;
import de.rwth.cnc.model.Port;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyIFCViolation;

import java.util.ArrayList;
import java.util.List;

public class CheckIFC extends Checker {
  private List<InconsistencyIFCViolation> ifcViolations;

  public CheckIFC(CnCArchitecture system, CnCView view) {
    this.system = system;
    this.view = view;
  }

  public List<InconsistencyIFCViolation> getIfcViolations() {
    return ifcViolations;
  }

  @Override
  public boolean checkConsistency() {
    ifcViolations = new ArrayList<>();

    List<Component> ifcMarkedComponents = new ArrayList<>();
    for (Component c : view.getComponents()) {
      if (c.isMarkedInterfaceComplete()) {
        ifcMarkedComponents.add(c);
      }
    }

    for (Component c : ifcMarkedComponents) {
      Component modelComponent = system.getComponent(c.getName());

      List<String> modelPortNameList = new ArrayList<>();
      List<String> viewPortNameList = new ArrayList<>();
      for (Port p : modelComponent.getPorts()) {
        modelPortNameList.add(p.getName());
      }
      for (Port p : c.getPorts()) {
        viewPortNameList.add(p.getName());
      }

      modelPortNameList.removeAll(viewPortNameList);

      for (String pn : modelPortNameList) {
        Port p = modelComponent.getPort(pn);
        ifcViolations.add(new InconsistencyIFCViolation(c.getName(), pn, p.getType(), p.getDirection()));
      }
    }

    return ifcViolations.isEmpty();
  }
}
