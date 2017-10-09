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
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyHierarchyMismatch;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyHierarchyMismatchKind;

/**
 * checks that every component directly contained in a view is really contained
 * in the architecture
 * does not check transitive children in the view to give a more succinct answer
 *
 * @author ringert
 */
public class CheckHierarchy extends Checker {

  private List<InconsistencyHierarchyMismatch> hierarchyMismatches;

  public CheckHierarchy(CnCArchitecture system, CnCView view) {
    this.system = system;
    this.view = view;
  }

  @Override
  public boolean checkConsistency() {

    hierarchyMismatches = new ArrayList<InconsistencyHierarchyMismatch>();

    Set<String> cmpsToCheck = new LinkedHashSet<String>();
    cmpsToCheck.addAll(view.getComponentNames());
    cmpsToCheck.retainAll(system.getComponentNames());

    for (String cmpView : cmpsToCheck) {
      for (String childNameView : view.getComponent(cmpView).getContainedComponents()) {
        checkContainmentInArchitecture(cmpView, childNameView);
      }
      checkIndependentnessInArchitecture(view.getComponent(cmpView).getContainedComponents());
    }
    // also check independence for top level components
    List<String> topCmpsToCheck = new ArrayList<String>();
    topCmpsToCheck.addAll(view.getTopLevelComponentNames());
    topCmpsToCheck.retainAll(system.getComponentNames());
    checkIndependentnessInArchitecture(topCmpsToCheck);

    return hierarchyMismatches.isEmpty();
  }

  private void checkIndependentnessInArchitecture(List<String> containedComponents) {
    for (int c1 = 0; c1 < containedComponents.size() - 1; c1++) {
      for (int c2 = c1 + 1; c2 < containedComponents.size(); c2++) {
        String indepCmp1 = containedComponents.get(c1);
        String indepCmp2 = containedComponents.get(c2);
        if (system.isParentChild(indepCmp1, indepCmp2) || system.isParentChild(indepCmp2, indepCmp1)) {
          InconsistencyHierarchyMismatch hm = new InconsistencyHierarchyMismatch(indepCmp1, indepCmp2, InconsistencyHierarchyMismatchKind.IND_VIEW_NOT_IND_ARCH);
          hierarchyMismatches.add(hm);
        }
      }
    }
  }

  private void checkContainmentInArchitecture(String cmpView, String childNameView) {
    if (system.getComponentNames().contains(childNameView)) {
      // child is in architecture
      if (!system.isParentChild(cmpView, childNameView)) {
        // no containment as in view
        if (system.isParentChild(childNameView, cmpView)) {
          // we have reverse containment
          InconsistencyHierarchyMismatch hm = new InconsistencyHierarchyMismatch(cmpView, childNameView, InconsistencyHierarchyMismatchKind.REV_CONTAINMENT);
          hierarchyMismatches.add(hm);
        }
        else {
          // we have independence in architecture
          InconsistencyHierarchyMismatch hm = new InconsistencyHierarchyMismatch(cmpView, childNameView, InconsistencyHierarchyMismatchKind.IND_ARCH_NOT_IND_VIEW);
          hierarchyMismatches.add(hm);
        }
      }
    }
  }

  public List<InconsistencyHierarchyMismatch> getHierarchyMismatches() {
    return hierarchyMismatches;
  }

}
