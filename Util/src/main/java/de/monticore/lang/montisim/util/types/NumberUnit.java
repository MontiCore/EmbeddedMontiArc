package de.monticore.lang.montisim.util.types;

import de.monticore.lang.numberunit._ast.ASTUnitNumber;

public class NumberUnit {
  private String numberUnit;
  private float number;
  private String unit;

  public NumberUnit(String numberUnit) {
    this.numberUnit = numberUnit.replace("'","").replace(" ","");

    int i = 0;
    while(i < this.numberUnit.length()) {
      if(Character.isDigit(this.numberUnit.charAt(i)) || this.numberUnit.charAt(i)=='.' || this.numberUnit.charAt(i)=='-') {
        i++;
      } else {
        break;
      }
    }

    this.number = Float.parseFloat(this.numberUnit.substring(0,i));
    this.unit = this.numberUnit.substring(i);
  }

  public NumberUnit(float number, String unit) {
    this.number = number;
    this.unit = unit;
    this.numberUnit = number+unit;
  }

  public NumberUnit(ASTUnitNumber astUN) {
    this.number = astUN.getNumber().get().floatValue();
    if(astUN.getUnit().isPresent()) {
      this.unit = astUN.getUnit().get().toString();
    } else {
      this.unit = "";
    }
    this.numberUnit = number+unit;
  }

  public float getNumber() {
    return number;
  }

  public String getUnit() {
    return unit;
  }

  // Careful: Non-decimal numbers still get a trailing zero: eg. 4 -> 4.0
  public String getNumberUnit() {
    return numberUnit;
  }
}