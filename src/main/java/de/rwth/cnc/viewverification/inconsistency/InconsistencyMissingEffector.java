/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

/**
 * for now this is just extending the missing connection type
 */
public class InconsistencyMissingEffector
    extends InconsistencyMissingConnection {

  public InconsistencyMissingEffector(String componentSource, String componentTarget, String portSource, String portTarget) {
    super(componentSource, componentTarget, portSource, portTarget);
  }

}
