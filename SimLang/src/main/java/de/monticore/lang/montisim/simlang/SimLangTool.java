package de.monticore.lang.montisim.simlang;

/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */

import java.io.IOException;
import java.util.Optional;

import de.monticore.lang.montisim.carlang._ast.ASTCar;
import de.monticore.lang.montisim.carlang._symboltable.CarLangLanguage;
import org.antlr.v4.runtime.RecognitionException;

import de.monticore.lang.montisim.simlang.symbolmap.SymbolMapCreator;
import de.monticore.lang.montisim.simlang._ast.*;
import de.monticore.lang.montisim.simlang._cocos.*;
import de.monticore.lang.montisim.simlang._parser.*;
import de.monticore.lang.montisim.simlang._symboltable.*;
import de.monticore.lang.montisim.simlang.cocos.*;
import de.monticore.lang.montisim.weather.cocos.*;
import de.monticore.io.paths.ModelPath;
/*import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;*/
import de.monticore.symboltable.*;
import de.monticore.ModelingLanguageFamily;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Path;
import java.nio.file.Paths;

import de.monticore.lang.montisim.simlang.adapter.GeneralSLAdapter;
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
    final SimLangLang lang = new SimLangLang();

    // parse the model and create the AST representation
    final ASTSimLangCompilationUnit ast = parse(model);
    Log.info(model + " parsed successfully!", SimLangTool.class.getName());



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
    //checkDefaultCoCos(ast);
    Log.info("Finished CoCos.", SimLangTool.class.getName());

    // setup the symbol table
    Scope modelTopScope = createSymbolTable(lang, ast);
    /*
    SymbolMapCreator data = new SymbolMapCreator();
    data.createFromAST(ast);
    System.out.println(data.getSymbolMap());
    GeneralSLAdapter adapter = new GeneralSLAdapter(data.getSymbolMap());
    System.out.println(adapter.getMapName());
    */
    //Grab data from symboltable
    Log.info("Grabbing symboltable content...", SimLangTool.class.getName());
    System.out.println(modelTopScope);
    System.out.println(modelTopScope.getSymbolsSize());
    System.out.println(modelTopScope.getSymbols());
    System.out.println(modelTopScope.getLocalSymbols());
    System.out.println(modelTopScope.getSubScopes());
    Optional<Symbol> aSymbol = modelTopScope.resolve("sim_type", SimulationTypeSymbol.KIND);
    if (aSymbol.isPresent()) {
      Log.info("Resolved symbol = " + aSymbol.get().toString(), SimLangTool.class.getName());
    }
    System.out.println(aSymbol);
    Log.info("Finished grabbing.", SimLangTool.class.getName());

    Log.info("Done.", SimLangTool.class.getName());
  }

  /**
   * Parse the model contained in the specified file.
   *
   * @param model - file to parse
   * @return
   */
  public static ASTSimLangCompilationUnit parse(String model) {
    try {
      SimLangParser parser = new SimLangParser();
      Optional<ASTSimLangCompilationUnit> optSimulation = parser.parse(model);

      if (!parser.hasErrors() && optSimulation.isPresent()) {
        return optSimulation.get();
      }
      Log.error("Model could not be parsed.");
    }
    catch (RecognitionException | IOException e) {
      Log.error("Failed to parse " + model, e);
    }
    return null;
  }
  //todo: finalize range+lambda -> fix cocos

  public static void checkDefaultCoCos(ASTSimLangCompilationUnit ast) {
    SimLangCoCoChecker defaultCoCos = new SimLangCoCoChecker();

    //DataStructure Checkers --Need to runs first
    defaultCoCos.addCoCo(new NoInfRangeChecker());

    //Simulation CoCos
    defaultCoCos.addCoCo(new RequiredSimulationEntries());
    defaultCoCos.addCoCo(new NoMultipleSimulationEntries());
    defaultCoCos.addCoCo(new SimulationRenderFrequencyChecker());
    defaultCoCos.addCoCo(new SimulationLoopFrequencyChecker());
    defaultCoCos.addCoCo(new SimulationDurationChecker());
    defaultCoCos.addCoCo(new MapNameChecker());
    defaultCoCos.addCoCo(new TimeChecker());
    defaultCoCos.addCoCo(new TimeoutChecker());
    //defaultCoCos.addCoCo(new GravityChecker());
    defaultCoCos.addCoCo(new PedestrianDensityChecker());

    //Weather CoCos
    defaultCoCos.addCoCo(new NoMultipleWeatherObjEntries());
    defaultCoCos.addCoCo(new TemperatureChecker());
    defaultCoCos.addCoCo(new PressureChecker());
    defaultCoCos.addCoCo(new HumidityChecker());
    defaultCoCos.addCoCo(new SightChecker());
    defaultCoCos.addCoCo(new PrecipitationAmountChecker());
    defaultCoCos.addCoCo(new WindStrengthChecker());
    defaultCoCos.addCoCo(new WindDirectionChecker());

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
  public static Scope createSymbolTable(SimLangLang lang, ASTSimLangCompilationUnit ast) {
    final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
    resolverConfiguration.addTopScopeResolvers(lang.getResolvers());

    GlobalScope globalScope = new GlobalScope(new ModelPath(), lang, resolverConfiguration);

    Optional<SimLangSymbolTableCreator> symbolTable = lang.getSymbolTableCreator(resolverConfiguration, globalScope);
    return symbolTable.get().createFromAST(ast);
  }

  public static Scope createSymbolTable(ModelingLanguageFamily lang, ASTSimLangCompilationUnit astSim, ASTCar astCar) {

    return null;
  }

  public static GeneralSLAdapter parseIntoContainer(String model) {
    final SimLangLang lang = new SimLangLang();
    final ASTSimLangCompilationUnit ast = parse(model);
    checkDefaultCoCos(ast);
    final Scope modelTopScope = createSymbolTable(lang, ast);
    return new GeneralSLAdapter(modelTopScope);
  }
  public static GeneralSLAdapter parseIntoContainer(String simModel, String carModel) {
    ModelingLanguageFamily family = new ModelingLanguageFamily();
    final SimLangLang simLang = new SimLangLang();
    //final CarLangLanguage carLang = new CarLangLanguage();

    family.addModelingLanguage(simLang);
    //family.addModelingLanguage(carLang);

    final ASTSimLangCompilationUnit astSim = parse(simModel);
    //final ASTCar astCar = ;

    checkDefaultCoCos(astSim);
    //checkCocos(astCar);

    final Scope modelTopScope = createSymbolTable(family, astSim, null);
    return new GeneralSLAdapter(modelTopScope);
  }
}
