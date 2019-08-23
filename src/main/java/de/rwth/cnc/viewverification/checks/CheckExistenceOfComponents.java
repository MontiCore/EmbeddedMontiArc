/* (c) https://github.com/MontiCore/monticore */
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
