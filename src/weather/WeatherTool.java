package weather;

/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */

import java.io.IOException;
import java.util.Optional;

import org.antlr.v4.runtime.RecognitionException;

import weather._ast.*;
import weather._cocos.*;
import weather._od.*;
import weather._parser.*;
import weather._symboltable.*;
import weather._visitor.*;
import si._ast.*;
import si._cocos.*;
import si._od.*;
import si._parser.*;
import si._symboltable.*;
import si._visitor.*;
//import weather.cocos.AtLeastOneInitialAndFinalState;
//import weather.cocos.AutomatonCoCos;
//import weather.cocos.StateNameStartsWithCapitalLetter;
//import weather.cocos.TransitionSourceExists;
//import weather.prettyprint.PrettyPrinter;
//import weather.visitors.CountStates;
import de.monticore.io.paths.ModelPath;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

/**
 * Main class for the Automaton DSL tool.
 *
 * @author (last commit) $Author$
 * @version $Revision$, $Date$
 */
public class WeatherTool {
  
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
    Log.info("Weather DSL Tool", WeatherTool.class.getName());
    Log.info("------------------", WeatherTool.class.getName());
    String model = args[0];
    
    // setup the language infrastructure
    //final WeatherLanguage lang = new WeatherLanguage();
    
    // parse the model and create the AST representation
    final ASTWeather ast = parse(model);
    Log.info(model + " parsed successfully!", WeatherTool.class.getName());
    
    // setup the symbol table
    //Scope modelTopScope = createSymbolTable(lang, ast);
    // can be used for resolving things in the model
    //Optional<Symbol> aSymbol = modelTopScope.resolve("temperature", TemperatureSymbol.KIND);
    //if (aSymbol.isPresent()) {
    //  Log.info("Resolved state symbol \"temperature\"; FQN = " + aSymbol.get().toString(),
    //      WeatherTool.class.getName());
    //}
    
    // execute default context conditions
    runDefaultCoCos(ast);
    
    // execute a custom set of context conditions
    //AutomatonCoCoChecker customCoCos = new AutomatonCoCoChecker();
    //customCoCos.addCoCo(new StateNameStartsWithCapitalLetter());
    //customCoCos.checkAll(ast);
    
    // analyze the model with a visitor
    //CountStates cs = new CountStates();
    //cs.handle(ast);
    //Log.info("The model contains " + cs.getCount() + " states.", WeatherTool.class.getName());
    
    // execute a pretty printer
    //PrettyPrinter pp = new PrettyPrinter();
    //pp.handle(ast);
    //Log.info("Pretty printing the parsed automaton into console:", WeatherTool.class.getName());
    //System.out.println(pp.getResult());
    
    System.out.println("Done.");
  }
  
  /**
   * Parse the model contained in the specified file.
   * 
   * @param model - file to parse
   * @return
   */
  public static ASTWeather parse(String model) {
    try {
      WeatherParser parser = new WeatherParser() ;
      Optional<ASTWeather> optWeather = parser.parse(model);
      
      if (!parser.hasErrors() && optWeather.isPresent()) {
        return optWeather.get();
      }
      Log.error("Model could not be parsed.");
    }
    catch (RecognitionException | IOException e) {
      Log.error("Failed to parse " + model, e);
    }
    return null;
  }
  
  /**
   * Create the symbol table from the parsed AST.
   * 
   * @param lang
   * @param ast
   * @return
   */
  public static Scope createSymbolTable(WeatherLanguage lang, ASTWeather ast) {
    final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
    resolverConfiguration.addTopScopeResolvers(lang.getResolvers());
    
    GlobalScope globalScope = new GlobalScope(new ModelPath(), lang, resolverConfiguration);
    
    Optional<WeatherSymbolTableCreator> symbolTable = lang.getSymbolTableCreator(
        resolverConfiguration, globalScope);
    return symbolTable.get().createFromAST(ast);
  }
  
  /**
   * Run the default context conditions {@link AtLeastOneInitialAndFinalState},
   * {@link TransitionSourceExists}, and
   * {@link StateNameStartsWithCapitalLetter}.
   * 
   * @param ast
   */
  public static void runDefaultCoCos(ASTWeather ast) {
    final WeatherCoCoChecker checker = new WeatherCoCoChecker();
    //checker.addCoCo(new AtLeastOneInitialAndFinalState());
    checker.checkAll(ast);
  }
  
}
