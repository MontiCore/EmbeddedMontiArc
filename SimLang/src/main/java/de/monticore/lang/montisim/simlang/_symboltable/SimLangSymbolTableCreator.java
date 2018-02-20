package de.monticore.lang.montisim.simlang._symboltable;

import de.se_rwth.commons.logging.Log;
import de.monticore.lang.montisim.simlang._ast.*;

import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.ImportStatement;

import de.se_rwth.commons.Names;
import de.monticore.lang.montisim.simlang.util.*;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.Deque;

public class SimLangSymbolTableCreator extends SimLangSymbolTableCreatorTOP {
  
  public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
    super(resolvingConfig, enclosingScope);
  }
  public SimLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
  }



  // Scope Symbols
  @Override
  public void visit(ASTSimLangCompilationUnit node) {
    //coCoChecker.checkAll(node);
    String packageQualifiedName = Names.getQualifiedName(node.getPackage());
    List<ImportStatement> imports = node.getImportStatements()
            .stream()
            .map(imprt -> {
                String qualifiedImport = Names.getQualifiedName(imprt.getImportList());
                return new ImportStatement(qualifiedImport, imprt.isStar());
            })
            .collect(Collectors.toList());
    ArtifactScope artifactScope = new ArtifactScope(packageQualifiedName, imports);
    putOnStack(artifactScope);
  }
  @Override
  public void endVisit(ASTSimLangCompilationUnit node) {
    removeCurrentScope();
  }
  
  @Override
  public void visit(final ASTSimulation node) {
    final SimulationSymbol simSymbol = new SimulationSymbol(node.getName());

    addToScopeAndLinkWithNode(simSymbol, node);
  }

  @Override
  public void endVisit(final ASTSimulation node) {
    removeCurrentScope();
  }
  
  @Override
  public void visit(final ASTWeather node) {/*
    System.out.println("Visiting a weather node!:");
    //System.out.println(node);
    String name;
    if(node.getFixedWeather().isPresent()) {
      name = "fixed";
    } else if(node.getSequenceWeather().isPresent()) {
      name = "sequence";
    } else if(node.getRandomWeather().isPresent()) {
      name = "random";
    } else { //assume node.getForecast is present
      name = "forecast";
    }
    final WeatherSymbol weatherSymbol = new WeatherSymbol(name);

    addToScopeAndLinkWithNode(weatherSymbol, node);*/
  }

  @Override
  public void endVisit(final ASTWeather node) {
    //removeCurrentScope();
  }
  
  @Override
  public void visit(final ASTChannel node) {/*
    System.out.println("Visiting a channel node!:");
    //System.out.println(node);
    final ChannelSymbol channelSymbol = new ChannelSymbol(node.getName());

    addToScopeAndLinkWithNode(channelSymbol, node);*/
  }

  @Override
  public void endVisit(final ASTChannel node) {
    //removeCurrentScope();
  }

  // Standard Symbols

  //Simulation
  public void visit(final ASTSimulationRenderFrequency node) {
    final SimulationRenderFrequencySymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new SimulationRenderFrequencySymbol("sim_render_frequency", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new SimulationRenderFrequencySymbol("sim_render_frequency", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new SimulationRenderFrequencySymbol("sim_render_frequency", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new SimulationRenderFrequencySymbol("sim_render_frequency", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTSimulationLoopFrequency node) {
    final SimulationLoopFrequencySymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new SimulationLoopFrequencySymbol("sim_loop_frequency", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new SimulationLoopFrequencySymbol("sim_loop_frequency", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new SimulationLoopFrequencySymbol("sim_loop_frequency", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new SimulationLoopFrequencySymbol("sim_loop_frequency", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTSimulationDuration node) {
    final SimulationDurationSymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new SimulationDurationSymbol("sim_duration", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new SimulationDurationSymbol("sim_duration", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new SimulationDurationSymbol("sim_duration", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new SimulationDurationSymbol("sim_duration", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTSimulationType node) {
    System.out.println("AHA!");
    final SimulationTypeSymbol symbol;
    switch(node.getSimType()) {
      case 0:
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.FIXED);
        break;
      case 1:
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.REALTIME);
        break;
      case 2:
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.MAXFPS);
        break;
      default:
        //Log.error("Something broke while visiting sim_type node.");
        symbol = new SimulationTypeSymbol("sim_type", SimLangEnums.SimulationTypes.FIXED);
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  @Override
  public void visit(final ASTTime node) {
    final TimeSymbol symbol;
    if(node.getSingleTime().isPresent()) {
      Time val = new Time(node.getSingleTime().get().getHours(),
                          node.getSingleTime().get().getMinutes(),
                          node.getSingleTime().get().getSeconds(),
                          node.getSingleTime().get().getMilliseconds());
      symbol = new TimeSymbol("time", new TimeAlternatives(val));
    }
    else {
      ArrayList<Time> vals = new ArrayList<>();
      for(ASTSingleTime ele : node.getTimeList().get().getSingleTimes()) {
        vals.add(new Time(ele.getHours(),
                          ele.getMinutes(),
                          ele.getSeconds(),
                          ele.getMilliseconds()));
      }
      symbol = new TimeSymbol("time", new TimeAlternatives(vals));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapPath node) {
    final MapPathSymbol symbol = new MapPathSymbol("map_path", node.getMapPath());
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapName node) {
    final MapNameSymbol symbol = new MapNameSymbol("map_name", (node.getMapName()+node.getFileFormat()));
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapHeight node) {
    final MapHeightSymbol symbol;
    if(node.getCustomHeight().isPresent()) {
      symbol = new MapHeightSymbol("map_height", new MapHeight(node.getCustomHeight()));
    }
    else {
      switch(node.getHeightMode()) {
        case 0:
          symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.FLAT));
          break;
        case 1:
          symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.RANDOM));
          break;
        default:
          //Log.error("Something went wrong when visiting height mode node");
          symbol = new MapHeightSymbol("map_height", new MapHeight(SimLangEnums.SimulationHeightModes.FLAT));
      }
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapOverlap node) {
    final MapOverlapSymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new MapOverlapSymbol("map_overlap", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new MapOverlapSymbol("map_overlap", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new MapOverlapSymbol("map_overlap", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new MapOverlapSymbol("map_overlap", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapSectorWidth node) {
    final MapSectorWidthSymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new MapSectorWidthSymbol("map_sector_width", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new MapSectorWidthSymbol("map_sector_width", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new MapSectorWidthSymbol("map_sector_width", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new MapSectorWidthSymbol("map_sector_width", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMapSectorHeight node) {
    final MapSectorHeightSymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new MapSectorHeightSymbol("map_sector_height", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new MapSectorHeightSymbol("map_sector_height", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new MapSectorHeightSymbol("map_sector_height", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new MapSectorHeightSymbol("map_sector_height", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTMaxSectorUsers node) {
    final MaxSectorUsersSymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new MaxSectorUsersSymbol("max_sector_users", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new MaxSectorUsersSymbol("max_sector_users", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new MaxSectorUsersSymbol("max_sector_users", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new MaxSectorUsersSymbol("max_sector_users", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTTimeout node) {
    final TimeoutSymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new TimeoutSymbol("timeout", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new TimeoutSymbol("timeout", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new TimeoutSymbol("timeout", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new TimeoutSymbol("timeout", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTGravity node) {
    final GravitySymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new GravitySymbol("gravity", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new GravitySymbol("gravity", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new GravitySymbol("gravity", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new GravitySymbol("gravity", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }
  public void visit(final ASTPedestrianDensity node) {
    final PedestrianDensitySymbol symbol;
    if(node.getTUnitNumber().isPresent()) {
      symbol = new PedestrianDensitySymbol("pedestrian_density", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getTUnitNumberList().isPresent()) {
      symbol = new PedestrianDensitySymbol("pedestrian_density", new ValueListRangeLambda(node.getTUnitNumber()));
    }
    else if(node.getRange().isPresent()) {
      symbol = new PedestrianDensitySymbol("pedestrian_density", new ValueListRangeLambda(node.getRange()));
    }
    else {
      symbol = new PedestrianDensitySymbol("pedestrian_density", new ValueListRangeLambda(node.getLambda()));
    }
    addToScopeAndLinkWithNode(symbol, node);
  }


}