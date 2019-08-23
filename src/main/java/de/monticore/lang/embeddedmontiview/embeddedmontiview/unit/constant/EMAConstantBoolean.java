/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview.unit.constant;

/**
 * Represents a constant Boolean which is used for constant connectors
 */
public class EMAConstantBoolean extends EMAConstantValue {
  public EMAConstantBoolean(boolean b) {
    super(b);
  }

  @Override
  public boolean isBoolean() {
    return true;
  }

  @Override
  public String getValueAsString() {
    return String.valueOf(getValue());
  }
}
