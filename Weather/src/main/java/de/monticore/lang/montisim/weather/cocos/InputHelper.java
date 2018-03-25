package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.montisim.weather._ast.ASTAlternativeInput;
import de.monticore.lang.numberunit._ast.ASTUnitNumber;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;

public class InputHelper {
  private ArrayList<NumberUnit> extractedValues;

  public InputHelper(ASTAlternativeInput obj) {
    this.extractedValues = new ArrayList<>();
    if(obj.unitNumberIsPresent()) {
      this.extractedValues.add(new NumberUnit(obj.getUnitNumber().get()));
    }
    else if(obj.unitNumberListIsPresent()){
      for(ASTUnitNumber num : obj.getUnitNumberList().get().getUnitNumbers()) {
        this.extractedValues.add(new NumberUnit(num));
      }
    }
    else if(obj.rangeIsPresent()) {
      this.extractedValues.add(new NumberUnit(obj.getRange().get().getStartValue().floatValue(),obj.getRange().get().getStartUnit().toString()));
      this.extractedValues.add(new NumberUnit(obj.getRange().get().getStepValue().floatValue(), obj.getRange().get().getStepUnit().toString()));
      this.extractedValues.add(new NumberUnit(obj.getRange().get().getEndValue().floatValue(), obj.getRange().get().getEndUnit().toString()));
    }
    else {
      Log.error("Input helper received a strange input.");
    }
  }
  
  public ArrayList<NumberUnit> getExtractedValues() {
    return this.extractedValues;
  }
}