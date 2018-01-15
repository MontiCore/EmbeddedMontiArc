/*
 * Custom CoCos for Gravity
 * @author Deniz Schmidt 
 * SE RWTH Aachen
 */
package simlang.cocos;

import simlang._ast.ASTGravity;
import si._ast.ASTUnitNumber;
import simlang._cocos.SimLangASTGravityCoCo;
import de.se_rwth.commons.logging.Log;
import simlang.cocos.UnitNumberChecker;
import java.util.Optional;
import java.util.ArrayList;

public class GravityChecker implements SimLangASTGravityCoCo {
  
  @Override
  public void check(ASTGravity obj) {
    System.out.println("[CoCo] GravityChecker...");
    
    String[] allowedUnits = {"m/s^2"};
    
    ArrayList<String> inputs = new ArrayList();
    /*
    if(obj.getTUnitNumber().isPresent()) {
      inputs.add(obj.getTUnitNumber().get());
    } 
    else if(obj.getRange().isPresent()) {
      inputs.add(obj.getRange().get().getStart().get().getUnitNumber().get().getTUnitNumber().get());
      inputs.add(obj.getRange().get().getStep().get().getUnitNumber().get().getTUnitNumber().get());
      inputs.add(obj.getRange().get().getEnd().get().getUnitNumber().get().getTUnitNumber().get());
    } 
    else if(obj.getTUnitNumberList().isPresent()){
      inputs.addAll(obj.getTUnitNumberList().get().getTUnitNumbers());
    } 
    else if(obj.getLambda().isPresent()) {
      
      //do lambda stuff
    } 
    else {
      Log.error("This isn't supposed to happen!.");
    }
    */
    InputHelper helper = new InputHelper(obj);
    inputs = helper.getExtractedValues();
    
    for(String value : inputs) {
      System.out.println("Checking: "+value);
      UnitNumberChecker checker = new UnitNumberChecker(value, allowedUnits);
      
      if(!checker.inPositiveRange()) {
        Log.error("Range Error: Gravity must be greater 0.");
      }
      if(!checker.legitUnit()) {
        Log.error("Unit Error: Gravity missing or invalid unit.");
      }
    }
    
    System.out.println("[Done] GravityChecker");
  }
  
}
