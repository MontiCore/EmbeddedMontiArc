/* (c) https://github.com/MontiCore/monticore */
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
