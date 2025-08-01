/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.inconsistency;

abstract public class Inconsistency {

  private String justificationFileName;

  public String getJustificationFileName() {
    return justificationFileName;
  }

  public void setJustificationFileName(String justificationFileName) {
    this.justificationFileName = justificationFileName;
  }
}
