/* (c) https://github.com/MontiCore/monticore */
/* generated by template templates.de.monticore.lang.tagschema.ValuedTagType*/


package nfp.TransmissionCostsTagSchema;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

import org.jscience.physics.amount.Amount;
import javax.measure.quantity.Power;
import javax.measure.unit.Unit;

/**
 * Created by ValuedTagType.ftl
 */
public class TransCostConnSymbol extends TagSymbol {
  public static final TransCostConnKind KIND = TransCostConnKind.INSTANCE;

  public TransCostConnSymbol(Amount<Power> value) {
    this(KIND, value);
  }

  public TransCostConnSymbol(Number number, Unit<Power> unit) {
    this(KIND, number, unit);
  }

  protected TransCostConnSymbol(TransCostConnKind kind, Amount<Power> value) {
    super(kind, value);
  }

  protected TransCostConnSymbol(TransCostConnKind kind, Number number, Unit<Power> unit) {
    this(kind, number.toString().contains(".") ?
      Amount.valueOf(number.doubleValue(), unit) :
      Amount.valueOf(number.intValue(),
          unit));
  }

  public Amount<Power> getValue() {
     return getValue(0);
  }

  public Number getNumber() {
    return getValue().getExactValue();
  }

  public Unit<Power> getUnit() {
    return getValue().getUnit();
  }

  @Override
  public String toString() {
    return String.format("TransCostConn = %s %s",
      getNumber(), getUnit());
  }

  public static class TransCostConnKind extends TagKind {
    public static final TransCostConnKind INSTANCE = new TransCostConnKind();

    protected TransCostConnKind() {
    }
  }
}
