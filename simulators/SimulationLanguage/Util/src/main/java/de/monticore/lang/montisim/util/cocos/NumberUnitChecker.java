/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.cocos;

import de.monticore.lang.montisim.util.types.NumberUnit;

public class NumberUnitChecker extends NumberChecker{
  private String unitNumber;
  private String[] allowedUnits;
  private String unit;
  
  public NumberUnitChecker(String unitNumber, String[] allowedUnits) {
    super(0.0f);
    this.allowedUnits = allowedUnits;
    this.unitNumber = unitNumber.replace("'","").replace(" ","");
    
    int i = 0;
    while(i < this.unitNumber.length()) {
      if(Character.isDigit(this.unitNumber.charAt(i)) || this.unitNumber.charAt(i)=='.' || this.unitNumber.charAt(i)=='-') {
        i++;
      } else {
        break;
      }
    }
    
    this.digit = Float.parseFloat(this.unitNumber.substring(0,i));
    this.unit = this.unitNumber.substring(i);
  }

  public NumberUnitChecker(NumberUnit nu, String[] allowedUnits) {
    super(0.0f);
    this.digit = nu.getNumber();
    this.unit = nu.getUnit();
    this.unitNumber = nu.getNumberUnit();
    this.allowedUnits = allowedUnits;
  }
  
  public String getUnit() {
    return this.unit;
  }
  
  public boolean legitUnit() {
    for(int i=0;i<this.allowedUnits.length;i++) {
      if(this.unit.equals(this.allowedUnits[i])) {
        return true;
      }
    }
    return false;
  }
}
