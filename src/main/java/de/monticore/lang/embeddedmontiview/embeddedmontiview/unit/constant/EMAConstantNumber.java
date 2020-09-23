/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview.unit.constant;

/**
 * Represents a constant Number which is used for constant connectors
 */
@Deprecated
public class EMAConstantNumber extends EMAConstantValue {
  public EMAConstantNumber(Number value) {
    super(value);
  }

  public Number getNumber() {
    return (Number) value;
  }

}
