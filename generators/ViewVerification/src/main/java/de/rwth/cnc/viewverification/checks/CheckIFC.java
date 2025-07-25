/* (c) https://github.com/MontiCore/monticore */
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
