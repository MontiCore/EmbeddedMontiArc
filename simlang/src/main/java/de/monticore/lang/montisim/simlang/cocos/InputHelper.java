package de.monticore.lang.montisim.simlang.cocos;

import java.util.Optional;
import java.util.ArrayList;
import de.se_rwth.commons.logging.Log;


public class InputHelper<T> {
  ArrayList<String> extractedValues;
  
  public InputHelper(T obj) {
    this.extractedValues = new ArrayList<String>();
    if(obj.getTUnitNumber().isPresent()) {
      this.extractedValues.add(obj.getTUnitNumber().get());
    } 
    else if(obj.getRange().isPresent()) {
      this.extractedValues.add(obj.getRange().get().getStart().get().getUnitNumber().get().getTUnitNumber().get());
      this.extractedValues.add(obj.getRange().get().getStep().get().getUnitNumber().get().getTUnitNumber().get());
      this.extractedValues.add(obj.getRange().get().getEnd().get().getUnitNumber().get().getTUnitNumber().get());
    } 
    else if(obj.getTUnitNumberList().isPresent()){
      this.extractedValues.addAll(obj.getTUnitNumberList().get().getTUnitNumbers());
    } 
    else if(obj.getLambda().isPresent()) {
      
      //do lambda stuff
    } 
    else {
      Log.error("This isn't supposed to happen!.");
    }
  }
  
  public ArrayList<String> getExtractedValues() {
    return this.extractedValues;
  }
}