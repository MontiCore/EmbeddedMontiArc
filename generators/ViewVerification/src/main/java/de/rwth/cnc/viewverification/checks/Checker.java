/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.checks;

import de.rwth.cnc.model.*;

public abstract class Checker {
  protected CnCView view;
  protected CnCArchitecture system;

  abstract public boolean checkConsistency();

}
