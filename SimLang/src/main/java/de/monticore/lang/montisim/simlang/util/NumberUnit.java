package de.monticore.lang.montisim.simlang.util;

public class NumberUnit {
  String numberUnit;
  float number;
  String unit;

  public NumberUnit(String numberUnit) {
    this.numberUnit = numberUnit.replace("'","").replace(" ","");

    int i = 0;
    while(i < this.numberUnit.length()) {
      if(Character.isDigit(this.numberUnit.charAt(i)) || this.numberUnit.charAt(i)=='.') {
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
  }

  public float getNumber() {
    return number;
  }

  public String getUnit() {
    return unit;
  }
}