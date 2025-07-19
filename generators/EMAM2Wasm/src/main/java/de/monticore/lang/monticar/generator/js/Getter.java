/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.js;

public class Getter {

  private String methodName;
  private String delegateMethodName;
  private String unit;

  public String getMethodName() {
    return methodName;
  }

  public void setMethodName(String methodName) {
    this.methodName = methodName;
  }

  public String getDelegateMethodName() {
    return delegateMethodName;
  }

  public void setDelegateMethodName(String delegateMethodName) {
    this.delegateMethodName = delegateMethodName;
  }

  public String getUnit() {
    return unit;
  }

  public void setUnit(String unit) {
    this.unit = unit;
  }
}
