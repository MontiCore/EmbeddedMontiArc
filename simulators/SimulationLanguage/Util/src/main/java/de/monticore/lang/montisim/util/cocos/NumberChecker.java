/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.cocos;

public class NumberChecker {
  protected float digit;
  
  public NumberChecker(String digit) {
    this.digit = Float.parseFloat(digit);
  }
  public NumberChecker(float digit) {
    this.digit = digit;
  }
  
  public float getDigit() {
    return this.digit;
  }
  
  public boolean inPositiveRange() {
    return this.digit > 0.0f;
  }
  
  public boolean inNegativeRange() {
    return this.digit < 0.0f;
  }
  
  //at least min
  public boolean inMinRange(float min) {
    return min <= this.digit;
  }
  
  //at most max
  public boolean inMaxRange(float max) {
    return this.digit <= max;
  }
  
  public boolean inClosedRange(float min, float max) {
    return (min <= this.digit)&&(this.digit <= max);
  }
  
  public boolean inOpenRange(float min, float max) {
    return (min < this.digit)&&(this.digit < max);
  }
  
  public boolean inHalfOpenRange(float open, float closed) {
    if(open < closed) {
      return open < this.digit && this.digit <= closed;
    } else {
      return closed <= this.digit && this.digit < open;
    }
  }
  
  public boolean noDecimals() {
    return (this.digit % 1) == 0.0f;
  }
}
