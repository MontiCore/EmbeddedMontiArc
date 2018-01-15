package weather.cocos;

public class UnitNumberChecker extends NumberChecker{
  private String unitNumber;
  private String[] allowedUnits;
  private String unit;
  
  public UnitNumberChecker(String unitNumber, String[] allowedUnits) {
    super(0.0f);
    this.allowedUnits = allowedUnits;
    this.unitNumber = unitNumber.replace("'","").replace(" ","");
    
    int i = 0;
    while(i < this.unitNumber.length()) {
      if(Character.isDigit(this.unitNumber.charAt(i)) || this.unitNumber.charAt(i)=='.') {
        i++;
      } else {
        break;
      }
    }
    
    setDigit(Float.parseFloat(this.unitNumber.substring(0,i)));
    this.unit = this.unitNumber.substring(i);
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