/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

public class InconsistencyItem {

  private InconsistencyKind inconsistencyKind;
  private String justificationFileName;
  private String justificationDescription;

  public InconsistencyItem(InconsistencyKind inconsistencyKind, String justificationFileName, String justificationDescription) {
    this.inconsistencyKind = inconsistencyKind;
    this.justificationFileName = justificationFileName;
    if (this.justificationFileName == null)
      this.justificationFileName = "";
    this.justificationDescription = justificationDescription;
  }

  public InconsistencyKind getInconsistencyKind() {
    return inconsistencyKind;
  }

  public String getJustificationFileName() {
    return justificationFileName;
  }

  public String getJustificationDescription() {
    return justificationDescription;
  }
}
