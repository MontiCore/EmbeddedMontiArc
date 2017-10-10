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

public class InconsistencyHierarchyMismatch extends Inconsistency {

  private String componentParent;
  private String componentChild;

  private InconsistencyHierarchyMismatchKind mismatchKind;

  public InconsistencyHierarchyMismatch(String componentParent, String componentChild, InconsistencyHierarchyMismatchKind mismatchKind) {
    this.componentParent = componentParent;
    this.componentChild = componentChild;
    this.mismatchKind = mismatchKind;
  }

  public String getComponentParent() {
    return componentParent;
  }

  public String getComponentChild() {
    return componentChild;
  }

  public InconsistencyHierarchyMismatchKind getMismatchKind() {
    return mismatchKind;
  }

}
