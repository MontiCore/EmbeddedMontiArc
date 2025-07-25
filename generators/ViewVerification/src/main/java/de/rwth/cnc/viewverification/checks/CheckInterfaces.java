/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.checks;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyInterfaceMismatch;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyInterfaceMismatchKind;

public class CheckInterfaces extends Checker {

  private List<InconsistencyInterfaceMismatch> interfaceMismatches;

  public CheckInterfaces(CnCArchitecture system, CnCView view) {
    this.system = system;
    this.view = view;
  }

  public List<InconsistencyInterfaceMismatch> getInterfaceMismatches() {
    return interfaceMismatches;
  }

  /**
   * checks consistency of interfaces and ignored components that are not part
   * of the architecture
   *
   * @return true if arch has interfaces as in the views
   */
  @Override
  public boolean checkConsistency() {

    interfaceMismatches = new ArrayList<InconsistencyInterfaceMismatch>();

    Set<String> cmpsToCheck = new LinkedHashSet<String>();
    cmpsToCheck.addAll(view.getComponentNames());
    cmpsToCheck.retainAll(system.getComponentNames());

    for (String cmpName : cmpsToCheck) {
      checkInterfaceOfCmp(cmpName);
    }

    return interfaceMismatches.isEmpty();
  }

  private void checkInterfaceOfCmp(String cmpName) {
    Component cmpView = view.getComponent(cmpName);
    Component cmpSys = system.getComponent(cmpName);
    if (cmpSys == null || cmpView == null) {
      // sanity if malformed model reports components from connectors that
      // are no real components in the system (e.g. typos)
      return;
    }
    List<Port> mappedPorts = new ArrayList<>();
    for (Port pView : cmpView.getPorts()) {
      String portType = pView.isUntyped() ? "" : pView.getType();
      String portName = pView.isUnnamed() ? "" : pView.getName();
      boolean incoming = pView.getDirection().equals(Direction.IN);

      if (pView.isUnnamed() || pView.isTextualAnonymous()) {
        boolean found = false;
        for (Port pSys : cmpSys.getPorts()) {
          if (mappedPorts.contains(pSys))  //port already has been used:
            continue;
          if (pView.getDirection().equals(pSys.getDirection())) {
            if (pView.isTyped()) {
              if (pView.getType().equals(pSys.getType()))
                found = true;
            }
            else
              found = true;
          }
          if (found) {
            mappedPorts.add(pSys);
            pView.setMappedPort(pSys);
            break;
          }
        }
        if (!found) {
          InconsistencyInterfaceMismatch im = new InconsistencyInterfaceMismatch(cmpName, portName, portType, incoming, InconsistencyInterfaceMismatchKind.NO_MATCH);
          interfaceMismatches.add(im);
        }
      }
      else {
        Port pSys = cmpSys.getPort(pView.getName());
        boolean error = false;
        if (pSys == null) {
          InconsistencyInterfaceMismatch im = new InconsistencyInterfaceMismatch(cmpName, portName, portType, incoming, InconsistencyInterfaceMismatchKind.NO_MATCH);
          interfaceMismatches.add(im);
          error = true;
        }
        else {
          if (!pView.getDirection().equals(pSys.getDirection())) {
            InconsistencyInterfaceMismatch im = new InconsistencyInterfaceMismatch(cmpName, portName, portType, incoming, InconsistencyInterfaceMismatchKind.WRONG_DIRECTION);
            interfaceMismatches.add(im);
            error = true;
          }
          if (!pView.isUntyped() && !pView.getType().equals(pSys.getType())) {
            InconsistencyInterfaceMismatch im = new InconsistencyInterfaceMismatch(cmpName, portName, portType, incoming, InconsistencyInterfaceMismatchKind.WRONG_TYPE);
            interfaceMismatches.add(im);
            error = true;
          }
        }

        if (!error) {
          mappedPorts.add(pSys);
          pView.setMappedPort(pSys);
        }
      }

    }
  }

}
