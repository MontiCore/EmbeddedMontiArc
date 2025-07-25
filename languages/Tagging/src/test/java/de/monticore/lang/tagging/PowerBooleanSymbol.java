/* (c) https://github.com/MontiCore/monticore */
/* generated from model null*/
/* generated by template templates.de.monticore.lang.montiarc.tagschema.ValuedTagType*/


package de.monticore.lang.montiarc.tagging;

import javax.measure.Measure;
import javax.measure.quantity.Power;
import javax.measure.unit.Unit;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

/**
 * Created by ValuedTagType.ftl
 */
public class PowerBooleanSymbol extends TagSymbol {
  public static final PowerBooleanKind KIND = PowerBooleanKind.INSTANCE;

  public PowerBooleanSymbol(Measure<? extends Number, Power> value) {
    this(KIND, value);
  }

  public PowerBooleanSymbol(Number number, Unit<Power> unit) {
    this(KIND, number, unit);
  }

  protected PowerBooleanSymbol(PowerBooleanKind kind, Measure<? extends Number, Power> value) {
    super(kind, value);
  }

  protected PowerBooleanSymbol(PowerBooleanKind kind, Number number, Unit<Power> unit) {
    this(kind, number.toString().contains(".") ?
        Measure.valueOf(number.doubleValue(), unit) :
        Measure.valueOf(number.intValue(),
            unit));
  }

  public Measure<? extends Number, Power> getValue() {
     return getValue(0);
  }

  public Unit<Power> getUnit() {
    return getValue().getUnit();
  }

  public Number getNumber() {
    return getValue().getValue();
  }

  @Override
  public String toString() {
    return String.format("PowerBoolean = %s",
      getValue().toString());
  }

  public static class PowerBooleanKind extends TagKind {
    public static final PowerBooleanKind INSTANCE = new PowerBooleanKind();

    protected PowerBooleanKind() {
    }
  }
}
