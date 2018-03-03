package de.monticore.lang.montisim.simlang.symbolmap;

import java.util.HashMap;
import java.util.ArrayList;

import de.monticore.lang.montisim.simlang._ast.*;
import de.monticore.lang.montisim.simlang._visitor.SimLangVisitor;
import de.monticore.lang.montisim.simlang.util.NumberUnit;
import de.monticore.lang.montisim.simlang.util.Range;
import de.monticore.lang.montisim.simlang.util.Lambda;
import de.monticore.lang.montisim.simlang.util.Time;
import de.monticore.lang.montisim.simlang.util.Channel;

import de.se_rwth.commons.logging.Log;



public class SymbolMapCreator implements SimLangVisitor{
/*
  HashMap<String, Object> symbolMap;

  private SimLangVisitor realThis = this;

  public SymbolMapCreator() {
    this.symbolMap = new HashMap<>();
  }

  public HashMap<String, Object> getSymbolMap() {
    return this.symbolMap;
  }

  public HashMap createFromAST(de.monticore.lang.montisim.simlang._ast.ASTSimLangNode rootNode) {
    Log.errorIfNull(rootNode, "Error by creating of the SymbolMap: top ast node is null");
    rootNode.accept(realThis);
    return this.symbolMap;
  }

  @Override
  public void visit(final ASTSimulationRenderFrequency node) {
    String id = "sim_render_frequency";
    if(node.getTUnitNumber().isPresent()) {
      NumberUnit value = new NumberUnit(node.getTUnitNumber().get());
      this.symbolMap.put(id, value);
    }
    else if(node.getRange().isPresent()) {
      Range value = new Range(null,null,null);
      this.symbolMap.put(id, value);
    }
    else if(node.getTUnitNumberList().isPresent()) {
      ArrayList<NumberUnit> values = new ArrayList<NumberUnit>();
      for(String nu : node.getTUnitNumberList().get().getTUnitNumbers()) {
        values.add(new NumberUnit(nu));
      }
      this.symbolMap.put(id, values);
    }
    else if(node.getLambda().isPresent()) {
      Lambda value = new Lambda(null); //todo: lambda
      this.symbolMap.put(id, value);
    }
    else {
      Log.error("Critical Error setting up the SymbolMap: #4000");
    }
  }
  @Override
  public void visit(final ASTSimulationLoopFrequency node) {
    String id = "sim_loop_frequency";
    if(node.getTUnitNumber().isPresent()) {
      NumberUnit value = new NumberUnit(node.getTUnitNumber().get());
      this.symbolMap.put(id, value);
    }
    else if(node.getRange().isPresent()) {
      Range value = new Range(null,null,null);
      this.symbolMap.put(id, value);
    }
    else if(node.getTUnitNumberList().isPresent()) {
      ArrayList<NumberUnit> values = new ArrayList<NumberUnit>();
      for(String nu : node.getTUnitNumberList().get().getTUnitNumbers()) {
        values.add(new NumberUnit(nu));
      }
      this.symbolMap.put(id, values);
    }
    else if(node.getLambda().isPresent()) {
      Lambda value = new Lambda(null); //todo: lambda
      this.symbolMap.put(id, value);
    }
    else {
      Log.error("Critical Error setting up the SymbolMap: #4000");
    }
  }
  @Override
  public void visit(final ASTSimulationDuration node) {
    String id = "sim_duration";
    if(node.getTUnitNumber().isPresent()) {
      NumberUnit value = new NumberUnit(node.getTUnitNumber().get());
      this.symbolMap.put(id, value);
    }
    else if(node.getRange().isPresent()) {
      Range value = new Range(null,null,null);
      this.symbolMap.put(id, value);
    }
    else if(node.getTUnitNumberList().isPresent()) {
      ArrayList<NumberUnit> values = new ArrayList<NumberUnit>();
      for(String nu : node.getTUnitNumberList().get().getTUnitNumbers()) {
        values.add(new NumberUnit(nu));
      }
      this.symbolMap.put(id, values);
    }
    else if(node.getLambda().isPresent()) {
      Lambda value = new Lambda(null); //todo: lambda
      this.symbolMap.put(id, value);
    }
    else {
      Log.error("Critical Error setting up the SymbolMap: #4000");
    }
  }
  @Override
  public void visit(final ASTSimulationType node) {
    String id="simulation_type";
    //System.out.println(node.getSimType());
    String value;
    switch(node.getSimType()) {
      case 1:
        value = "fixed";
        break;
      case 2:
        value = "realtime";
        break;
      case 3 : //todo: fixed = 2, realtime = 9, maxfps = 6????????
        value = "maxfps";
        break;
      default:
        value = "wtf";
    }
    this.symbolMap.put(id, value);

  }
  @Override
  public void visit(final ASTTime node) {
    String id = "time";
    Time value = null;//new Time(node.getHours(), node.getMinutes(), node.getSeconds(), node.getMilliseconds());
    this.symbolMap.put(id, value);
  }
  @Override
  public void visit(final ASTMapPath node) {
    String id = "map_path";
    String value = node.getMapPath();
    this.symbolMap.put(id, value);
  }
  @Override
  public void visit(final ASTMapName node) {
    String id = "map_name";
    String value = node.getMapName() + "." + node.getFileFormat();
    this.symbolMap.put(id, value);
  }
  @Override
  public void visit(final ASTMapHeight node) {
    String id = "map_height";
    //if(node.)
  }
  @Override
  public void visit(final ASTMapOverlap node) {
    String id = "map_overlap";
  }
  @Override
  public void visit(final ASTMapSectorWidth node) {
    String id = "map_sector_width";
  }
  @Override
  public void visit(final ASTMapSectorHeight node) {
    String id = "map_sector_height";
  }
  @Override
  public void visit(final ASTMaxSectorUsers node) {
    String id = "max_sector_users";
  }
  @Override
  public void visit(final ASTTimeout node) {
    String id = "timeout";
  }
  @Override
  public void visit(final ASTGravity node) {
    String id = "gravity";
    if(node.getTUnitNumber().isPresent()) {
      NumberUnit value = new NumberUnit(node.getTUnitNumber().get());
      this.symbolMap.put(id, value);
    }
    else if(node.getRange().isPresent()) {
      Range value = new Range(null,null,null);
      this.symbolMap.put(id, value);
    }
    else if(node.getTUnitNumberList().isPresent()) {
      ArrayList<NumberUnit> values = new ArrayList<NumberUnit>();
      for(String nu : node.getTUnitNumberList().get().getTUnitNumbers()) {
        values.add(new NumberUnit(nu));
      }
      this.symbolMap.put(id, values);
    }
    else if(node.getLambda().isPresent()) {
      Lambda value = new Lambda(null); //todo: lambda
      this.symbolMap.put(id, value);
    }
    else {
      Log.error("Critical Error setting up the SymbolMap: #4000");
    }
  }
  @Override
  public void visit(final ASTPedestrianDensity node) {
    String id = "pedestrian_density";
    if(node.getTUnitNumber().isPresent()) {
      NumberUnit value = new NumberUnit(node.getTUnitNumber().get());
      this.symbolMap.put(id, value);
    }
    else if(node.getRange().isPresent()) {
      Range value = new Range(null,null,null);
      this.symbolMap.put(id, value);
    }
    else if(node.getTUnitNumberList().isPresent()) {
      ArrayList<NumberUnit> values = new ArrayList<NumberUnit>();
      for(String nu : node.getTUnitNumberList().get().getTUnitNumbers()) {
        values.add(new NumberUnit(nu));
      }
      this.symbolMap.put(id, values);
    }
    else if(node.getLambda().isPresent()) {
      Lambda value = new Lambda(null); //todo: lambda
      this.symbolMap.put(id, value);
    }
    else {
      Log.error("Critical Error setting up the SymbolMap: #4000");
    }
  }
  @Override
  public void visit(final ASTChannel node) {
    if(!this.symbolMap.containsKey("channels")) {
      String id = "channels";
      ArrayList<Channel> value = new ArrayList();
      this.symbolMap.put(id, value);
    }
    //this.symbolMap.get("channels").add(new Channel());
  }*/
}
