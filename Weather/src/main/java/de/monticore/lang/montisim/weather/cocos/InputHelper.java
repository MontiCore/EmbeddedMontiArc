package de.monticore.lang.montisim.weather.cocos;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.montisim.weather._ast.ASTTUnitNumberList;
import de.monticore.lang.montisim.weather._ast.ASTAlternativeInput;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Optional;

public class InputHelper {
  private ArrayList<String> extractedValues;
  
  public InputHelper(Optional obj) {
    this.extractedValues = new ArrayList<>();
    if(obj.get() instanceof String) {
      this.extractedValues.add((String)obj.get());
    } 
    else if(obj.get() instanceof ASTRange) {
      this.extractedValues.add(((ASTRange) obj.get()).getStart().get().toString());
      this.extractedValues.add(((ASTRange) obj.get()).getStep().get().toString());
      this.extractedValues.add(((ASTRange) obj.get()).getEnd().get().toString());
    } 
    else if(obj.get() instanceof ASTTUnitNumberList){
      this.extractedValues.addAll(((ASTTUnitNumberList) obj.get()).getTUnitNumbers());
    }
    else {
      Log.error("Input helper received a strange input.");
    }
  }

  public InputHelper(ASTAlternativeInput obj) {
    this.extractedValues = new ArrayList<>();
    if(obj.tUnitNumberIsPresent()) {
      this.extractedValues.add(obj.getTUnitNumber().get());
    }
    else if(obj.tUnitNumberListIsPresent()){
      this.extractedValues.addAll(obj.getTUnitNumberList().get().getTUnitNumbers());
    }
    else if(obj.rangeIsPresent()) {
      this.extractedValues.add(obj.getRange().get().getStartValue().toString()+obj.getRange().get().getStartUnit().toString());
      this.extractedValues.add(obj.getRange().get().getStepValue().toString()+obj.getRange().get().getStepUnit().toString());
      this.extractedValues.add(obj.getRange().get().getEndValue().toString()+obj.getRange().get().getEndUnit().toString());
    }
    else {
      Log.error("Input helper received a strange input.");
    }
  }
  
  public ArrayList<String> getExtractedValues() {
    return this.extractedValues;
  }
}