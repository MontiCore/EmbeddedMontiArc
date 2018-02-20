package de.monticore.lang.montisim.simlang.cocos;

import java.util.Optional;
import java.util.ArrayList;

import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.montisim.simlang._ast.ASTTUnitNumberList;
import de.se_rwth.commons.logging.Log;

public class InputHelper {
  ArrayList<String> extractedValues;
  
  public InputHelper(Optional obj) {
    this.extractedValues = new ArrayList<>();
    if(obj.get() instanceof String) {
      this.extractedValues.add((String)obj.get());
    } 
    else if(obj.get() instanceof de.monticore.lang.monticar.ranges._ast.ASTRange) {
      this.extractedValues.add(((ASTRange) obj.get()).getStart().get().toString());
      this.extractedValues.add(((ASTRange) obj.get()).getStep().get().toString());
      this.extractedValues.add(((ASTRange) obj.get()).getEnd().get().toString());
    } 
    else if(obj.get() instanceof ASTTUnitNumberList){
      this.extractedValues.addAll(((ASTTUnitNumberList) obj.get()).getTUnitNumbers());
    } 
    else if(false) {
      
      //do lambda stuff
    } 
    else {
      Log.error("Input helper received a Optional, that was not meant for it.");
    }
  }
  
  public ArrayList<String> getExtractedValues() {
    return this.extractedValues;
  }
}