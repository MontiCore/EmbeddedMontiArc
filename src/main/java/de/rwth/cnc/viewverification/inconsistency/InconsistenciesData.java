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

import java.util.ArrayList;
import java.util.List;

import de.rwth.cnc.model.CnCArchitecture;
import de.rwth.cnc.model.CnCView;

public class InconsistenciesData {

  private CnCView view;
  private CnCArchitecture model;

  private List<InconsistencyMissingComponent> missingComponents = new ArrayList<>();
  private List<InconsistencyHierarchyMismatch> hierarchyMismatches = new ArrayList<>();
  private List<InconsistencyInterfaceMismatch> interfaceMismatches = new ArrayList<>();
  private List<InconsistencyMissingConnection> missingConnections = new ArrayList<>();
  private List<InconsistencyMissingEffector> missingEffectors = new ArrayList<>();
  private List<InconsistencyNotAtomic> notAtomicMismatches = new ArrayList<>();
  private List<InconsistencyIFCViolation> ifcViolations = new ArrayList<>();

  public void setView(CnCView view) {
    this.view = view;
  }

  public CnCView getView() {
    return view;
  }

  public void setModel(CnCArchitecture model) {
    this.model = model;
  }

  public CnCArchitecture getModel() {
    return model;
  }

  public void addMissingComponent(InconsistencyMissingComponent missingComponentName) {
    missingComponents.add(missingComponentName);
  }

  public void addHierarchyMismatch(InconsistencyHierarchyMismatch ihm) {
    hierarchyMismatches.add(ihm);
  }

  public void addInterfaceMismatch(InconsistencyInterfaceMismatch iim) {
    interfaceMismatches.add(iim);
  }

  public void addMissingConnection(InconsistencyMissingConnection imc) {
    missingConnections.add(imc);
  }

  public void addMissingEffector(InconsistencyMissingEffector imc) {
    missingEffectors.add(imc);
  }

  public void addNotAtomicMismatch(InconsistencyNotAtomic ina) {
    notAtomicMismatches.add(ina);
  }

  public void addIFCViolation(InconsistencyIFCViolation iifcv) {
    ifcViolations.add(iifcv);
  }

  public List<InconsistencyMissingComponent> getMissingComponents() {
    return missingComponents;
  }

  public List<InconsistencyHierarchyMismatch> getHierarchyMismatches() {
    return hierarchyMismatches;
  }

  public List<InconsistencyInterfaceMismatch> getInterfaceMismatches() {
    return interfaceMismatches;
  }

  public List<InconsistencyMissingConnection> getMissingConnections() {
    return missingConnections;
  }

  public List<InconsistencyMissingEffector> getMissingEffectors() {
    return missingEffectors;
  }

  public List<InconsistencyNotAtomic> getNotAtomicMismatches() {
    return notAtomicMismatches;
  }

  public List<InconsistencyIFCViolation> getIFCViolations() {
    return ifcViolations;
  }

  public boolean hasEntries() {
    return missingComponents.size() > 0 || hierarchyMismatches.size() > 0 || interfaceMismatches.size() > 0 || missingConnections.size() > 0 || missingEffectors.size() > 0 || notAtomicMismatches.size() > 0 || ifcViolations.size() > 0;
  }
}
