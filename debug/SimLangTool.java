package simlang;

/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */

import java.io.IOException;
import java.util.Optional;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.antlr.v4.runtime.RecognitionException;

import weather._ast.*;
import weather._cocos.*;
import weather._parser.*;
import weather._symboltable.*;
import weather._visitor.*;
import simlang._ast.*;
import simlang._cocos.*;
import simlang._parser.*;
import simlang._symboltable.*;
import simlang._visitor.*;
import numberunit._ast.*;
import numberunit._parser.*;
//import si._symboltable.*;
import numberunit._visitor.*;
import simlang.cocos.*;
import weather.cocos.*;
import de.monticore.io.paths.ModelPath;
/*import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;*/
import de.monticore.symboltable.*;
import de.se_rwth.commons.logging.Log;

/**
 * Main class for the Automaton DSL tool.
 *
 * @author (last commit) $Author$
 * @version $Revision$, $Date$
 */
public class SimLangTool {
  
  /**
   * Use the single argument for specifying the single input automaton file.
   * 
   * @param args
   */
  public static void main(String[] args) {
    if (args.length != 1) {
      Log.error("Please specify only one single path to the input model.");
      return;
    }
    Log.info("Simulation DSL Tool", SimLangTool.class.getName());
    Log.info("------------------", SimLangTool.class.getName());
    String model = args[0];
    
    // setup the language infrastructure
    final SimLangLanguage lang = new SimLangLanguage();
    
    // parse the model and create the AST representation
    final ASTSimulation ast = parse(model);
    Log.info(model + " parsed successfully!", SimLangTool.class.getName());
    
    // setup the symbol table
    //Scope modelTopScope = createSymbolTable(lang, ast);
    // can be used for resolving things in the model
    //Optional<String> scopeName = modelTopScope.getName();
    //List<?> scopeMap = modelTopScope.getSubScopes();
    //Optional<Symbol> aSymbol = modelTopScope.resolve("FullExample", SimulationKind.KIND);
    /*if (aSymbol.isPresent()) {
      Log.info("Resolved state symbol \"bla\"; FQN = " + aSymbol.get().toString(),
          SimLangTool.class.getName());
    }*/
    /*
    final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
    resolverConfiguration.addTopScopeResolvers(lang.getResolvers());
    
    Path mp = Paths.get(args[0]);
    GlobalScope globalScope = new GlobalScope(new ModelPath(mp), lang, resolverConfiguration);
    
    Optional<SimulationSymbolTableCreatorImp> symbolTable = lang.getSymbolTableCreator(resolverConfiguration, globalScope);
    Scope modelTopScope = symbolTable.get().createFromAST(ast);
    System.out.println(modelTopScope);
    */
    //Run Context-Conditions
    Log.info("Running CoCos...", SimLangTool.class.getName());
    checkDefaultCoCos(ast);
    Log.info("Finished CoCos.", SimLangTool.class.getName());
    
    //Grab data from symboltable
    Log.info("Grabbing symboltable content...", SimLangTool.class.getName());
    //System.out.println(modelTopScope);
    //System.out.println(scopeMap);
    Log.info("Finished grabbing.", SimLangTool.class.getName());
    
    Log.info("Done.", SimLangTool.class.getName());
  }
  
  /**
   * Parse the model contained in the specified file.
   * 
   * @param model - file to parse
   * @return
   */
  public static ASTSimulation parse(String model) {
    try {
      SimLangParser parser = new SimLangParser();
      Optional<ASTSimLangCompilationUnit> optSimulation = parser.parse(model);
      
      if (!parser.hasErrors() && optSimulation.isPresent()) {
        return optSimulation.get().getSimulation();
      }
      Log.error("Model could not be parsed.");
    }
    catch (RecognitionException | IOException e) {
      Log.error("Failed to parse " + model, e);
    }
    return null;
  }
  
  public static void checkDefaultCoCos(ASTSimulation ast) {
    SimLangCoCoChecker defaultCoCos = new SimLangCoCoChecker();
    
    //Simulation CoCos
    defaultCoCos.addCoCo(new RequiredSimulationEntries());
    defaultCoCos.addCoCo(new NoMultipleSimulationEntries());
    defaultCoCos.addCoCo(new SimulationRenderFrequencyChecker());
    defaultCoCos.addCoCo(new SimulationLoopFrequencyChecker());
    defaultCoCos.addCoCo(new SimulationDurationChecker());
    defaultCoCos.addCoCo(new TimeChecker());
    defaultCoCos.addCoCo(new TimeoutChecker());
    defaultCoCos.addCoCo(new GravityChecker());
    defaultCoCos.addCoCo(new PedestrianDensityChecker());
    
    //Weather CoCos
    defaultCoCos.addCoCo(new NoMultipleWeatherObjEntries());
    defaultCoCos.addCoCo(new TemperatureChecker());
    defaultCoCos.addCoCo(new PressureChecker());
    defaultCoCos.addCoCo(new HumidityChecker());
    defaultCoCos.addCoCo(new SightChecker());
    defaultCoCos.addCoCo(new PrecipitationamountChecker());
    defaultCoCos.addCoCo(new WindstrengthChecker());
    defaultCoCos.addCoCo(new WinddirectionChecker());
    
    //Communication CoCos
    defaultCoCos.addCoCo(new OneChannelEntryEach());
    defaultCoCos.addCoCo(new TransferrateChecker());
    defaultCoCos.addCoCo(new LatencyChecker());
    defaultCoCos.addCoCo(new OutageChecker());
    defaultCoCos.addCoCo(new AreaChecker());
    defaultCoCos.addCoCo(new CoordinateChecker());
    
    defaultCoCos.checkAll(ast);
  }
  
  /**
   * Create the symbol table from the parsed AST.
   * 
   * @param lang
   * @param ast
   * @return
   */
  public static Scope createSymbolTable(SimLangLanguage lang, ASTSimulation ast) {
    final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
    resolverConfiguration.addTopScopeResolvers(lang.getResolvers());
    
    GlobalScope globalScope = new GlobalScope(new ModelPath(), lang, resolverConfiguration);
    
    Optional<SimLangSymbolTableCreator> symbolTable = lang.getSymbolTableCreator(resolverConfiguration, globalScope);
    return symbolTable.get().createFromAST(ast);
  }
}
