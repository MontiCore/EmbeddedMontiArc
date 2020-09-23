/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview.unit.constant;

/**
 * Represents a constant String which is used for ConstantConnectors
 */
@Deprecated
public class EMAConstantString extends EMAConstantValue {
  public EMAConstantString(String value) {
    super(value);
  }

  public String getString() {
    return (String) value;
  }

  @Override
  public boolean isString() {
    return true;
  }
}
