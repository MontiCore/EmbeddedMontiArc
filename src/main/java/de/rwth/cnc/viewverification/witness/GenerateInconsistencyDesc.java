/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.witness;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.inconsistency.*;

public class GenerateInconsistencyDesc {

  public static String getDescForMissingComponent(String architectureName, String viewName, InconsistencyMissingComponent cmpName) {

    String desc;

    desc = "The component " + cmpName.getComponentName() + " of the view " + viewName + " is missing in the C&C model " + architectureName;

    return desc;
  }

  public static String getDescForHierarchyMismatch(String architectureName, String viewName, InconsistencyHierarchyMismatch hierarchyMismatch) {

    String desc = new String();

    switch (hierarchyMismatch.getMismatchKind()) {
      case IND_ARCH_NOT_IND_VIEW:
        desc = "The components " + hierarchyMismatch.getComponentParent() + " and " + hierarchyMismatch.getComponentChild() + " are independent in the C&C model " + architectureName + " but not independent in the view " + viewName;
        break;
      case IND_VIEW_NOT_IND_ARCH:
        desc = "The components " + hierarchyMismatch.getComponentParent() + " and " + hierarchyMismatch.getComponentChild() + " are independent in the view " + viewName + " but not independent in the C&C model " + architectureName;
        break;
      case REV_CONTAINMENT:
        desc = "The component " + hierarchyMismatch.getComponentParent() + " contains the component " + hierarchyMismatch.getComponentChild() + " in the view " + viewName + " but the latter contains the former in the C&C model " + architectureName;
        break;
    }

    return desc;
  }

  public static String getDescForInterfaceMismatch(String architectureName, String viewName, InconsistencyInterfaceMismatch interfaceMismatch) {

    String desc = new String();

    // sanitize port name by removing prefix
    String pType = interfaceMismatch.getPortType();
    if (pType != null && pType.startsWith(CnCViewConstants.TYPENAME_PREFIX)) {
      pType = pType.substring(CnCViewConstants.TYPENAME_PREFIX.length());
    }

    String portName = (interfaceMismatch.getPortName().isEmpty()) ? "unnamed port" : "port " + interfaceMismatch.getPortName();

    switch (interfaceMismatch.getMismatchKind()) {
      case WRONG_DIRECTION:
        String direction = interfaceMismatch.getIncoming() ? "incoming" : "outgoing";
        desc = "Wrong direction for port " + interfaceMismatch.getPortName() + " of component " + interfaceMismatch.getComponentName() + " (" + direction + ")";
        break;
      case WRONG_TYPE:
        desc = "Wrong type for port " + interfaceMismatch.getPortName() + " of component " + interfaceMismatch.getComponentName() + " (" + pType + ")";
        break;
      case NO_MATCH:
        desc = "No match for " + portName + " of component " + interfaceMismatch.getComponentName();
        break;
    }

    return desc;
  }

  public static String getDescForMissingConnection(String architectureName, String viewName, InconsistencyMissingConnection missingConnection) {

    String desc = new String();

    String portSource = missingConnection.getPortSource();
    portSource = (portSource == null || portSource.isEmpty()) ? "unnamed port" : "port " + missingConnection.getPortSource();

    String portTarget = missingConnection.getPortTarget();
    portTarget = (portTarget == null || portTarget.isEmpty()) ? "unnamed port" : "port " + missingConnection.getPortTarget();

    desc = "The C&C model " + architectureName + " is missing a connection from the component " + missingConnection.getComponentSource() + " to the component " + missingConnection.getComponentTarget() + " (from " + portSource + " to " + portTarget + ").";

    return desc;
  }

  public static String getDescForMissingEffector(String archName, String viewName, InconsistencyMissingEffector missingConnection) {

    String desc = new String();

    String portSource = missingConnection.getPortSource();
    portSource = (portSource == null || portSource.isEmpty()) ? "unnamed port" : "port " + missingConnection.getPortSource();

    String portTarget = missingConnection.getPortTarget();
    portTarget = (portTarget == null || portTarget.isEmpty()) ? "unnamed port" : "port " + missingConnection.getPortTarget();

    desc = "The C&C model " + archName + " is missing an effector from the component " + missingConnection.getComponentSource() + " to the component " + missingConnection.getComponentTarget() + " (from " + portSource + " to " + portTarget + ").";

    return desc;
  }

  public static String getDescForNotAtomicMismatch(String archName, String viewName, InconsistencyNotAtomic inconsistencyNotAtomic) {
    String desc = "The component " + inconsistencyNotAtomic.getComponentName() + " is marked atomic in the view " + viewName + " but is not atomic" + " in the C&C model " + archName + ".";
    return desc;
  }

  public static String getDescForIFCViolation(String archName, String viewName, InconsistencyIFCViolation inconsistencyIFCViolation) {
    String desc = "The port " + inconsistencyIFCViolation.getPortName() + " is not part of the component " + inconsistencyIFCViolation.getComponentName() + " which is marked as interface-complete in the view " + viewName + ".";
    return desc;
  }
}
