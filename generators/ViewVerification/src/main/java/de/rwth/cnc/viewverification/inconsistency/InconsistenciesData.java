/* (c) https://github.com/MontiCore/monticore */
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
