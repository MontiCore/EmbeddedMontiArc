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
import de.rwth.cnc.viewverification.inconsistency.InconsistencyNotAtomic;

import java.util.ArrayList;
import java.util.List;

public class CheckAtomic extends Checker {

  private List<InconsistencyNotAtomic> notAtomicMismatches;

  public CheckAtomic(CnCArchitecture system, CnCView view) {
    this.system = system;
    this.view = view;
  }

  public List<InconsistencyNotAtomic> getNotAtomicMismatches() {
    return notAtomicMismatches;
  }

  @Override
  public boolean checkConsistency() {
    notAtomicMismatches = new ArrayList<>();

    List<Component> atomicMarkedComponents = new ArrayList<>();
    for (Component c : view.getComponents()) {
      if (c.isMarkedAtomic())
        atomicMarkedComponents.add(c);
    }

    for (Component c : atomicMarkedComponents) {
      Component modelComponent = system.getComponent(c.getName());
      if (!modelComponent.isMarkedAtomic())
        notAtomicMismatches.add(new InconsistencyNotAtomic(modelComponent.getName(), modelComponent.getContainedComponents()));
    }

    return notAtomicMismatches.isEmpty();
  }
}
