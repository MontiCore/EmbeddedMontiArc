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

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import de.rwth.cnc.model.*;

public class CheckExistenceOfComponents extends Checker {

  private List<String> missingComponents;

  public CheckExistenceOfComponents(CnCArchitecture system, CnCView view) {
    this.system = system;
    this.view = view;
  }

  public List<String> getMissingComponents() {
    return missingComponents;
  }

  /**
   * @return true if arch contains all components in the view
   */
  @Override
  public boolean checkConsistency() {

    this.missingComponents = new ArrayList<String>();

    Set<String> cmpNamesArch = system.getComponentNames();
    Set<String> cmpNamesView = view.getComponentNames();

    this.missingComponents = new ArrayList<String>();

    if (!cmpNamesArch.containsAll(cmpNamesView)) {
      missingComponents.addAll(cmpNamesView);
      missingComponents.removeAll(cmpNamesArch);
      return false;
    }

    return true;
  }

}
