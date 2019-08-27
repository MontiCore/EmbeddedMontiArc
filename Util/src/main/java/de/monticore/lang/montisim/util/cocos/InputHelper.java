/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.cocos;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.lang.montisim.util.types.NumberUnit;
import de.se_rwth.commons.logging.Log;


import java.util.ArrayList;

public class InputHelper {
  private ArrayList<NumberUnit> extractedValues;

  public InputHelper(AlternativeInput obj) {
    this.extractedValues = new ArrayList<>();
    if(obj.getNUnit().isPresent()) {
      this.extractedValues.add(obj.getNUnit().get());
    }
    else if(obj.getList().isPresent()) {
      this.extractedValues = obj.getList().get();
    }
    else if(obj.getRange().isPresent()) {
      this.extractedValues.add(obj.getRange().get().getStart());
      this.extractedValues.add(obj.getRange().get().getStep());
      this.extractedValues.add(obj.getRange().get().getEnd());
    }
    else {
      Log.warn("Detected input is neither singular, list or range?!");
    }
  }
  
  public ArrayList<NumberUnit> getExtractedValues() {
    return this.extractedValues;
  }
}
