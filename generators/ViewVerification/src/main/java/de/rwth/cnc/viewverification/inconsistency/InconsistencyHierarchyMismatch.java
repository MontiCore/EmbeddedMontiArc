/* (c) https://github.com/MontiCore/monticore */
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
