/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview.unit.constant;

/**
 * Represents a constant Char which is used for constant connectors
 */
@Deprecated
public class EMAConstantChar extends EMAConstantValue {
  public EMAConstantChar(char value) {
    super(value);
  }

  public char getChar() {
    return (Character) value;
  }

  @Override
  public boolean isChar() {
    return true;
  }
}
