package de.monticore.lang.montisim.simlang;

/*
 * Copyright (c) 2017 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */

import java.io.IOException;
import java.util.Optional;

//import de.monticore.lang.montisim.carlang._ast.ASTCar;
//import de.monticore.lang.montisim.carlang._symboltable.CarLangLanguage;
import de.monticore.lang.montisim.simlang.adapter.SimLangContainer;
import org.antlr.v4.runtime.RecognitionException;

import de.monticore.lang.montisim.simlang._ast.ASTSimLangCompilationUnit;
import de.monticore.lang.montisim.simlang._cocos.SimLangCoCoChecker;
import de.monticore.lang.montisim.simlang._parser.SimLangParser;
import de.monticore.lang.montisim.simlang._symboltable.*;
import de.monticore.lang.montisim.simlang.cocos.*;
import de.monticore.lang.montisim.weather.cocos.*;
import de.monticore.io.paths.ModelPath;
/*import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;*/
import de.monticore.symboltable.*;
//import de.monticore.ModelingLanguageFamily;
import de.se_rwth.commons.logging.Log;

/**
 * Main class for the SimLang DSL tool.
 *
 * @author Schmidt, Deniz
 * @version 1.0.1
 */
public class SimLangTool {
  public static SimLangLang SIMLANG_LANGUAGE = new SimLangLang();

  /**
   * Use the single argument for specifying the single input automaton file.
   *
   * @param args
   */
  public static void main(String[] args) {
    if (args.length != 1) {
      Log.warn("Please specify only one single path to the input model.");
      return;
    }
    Log.info("SimLang-DSL Tool", SimLangTool.class.getName());
    Log.info("------------------", SimLangTool.class.getName());
    String model = args[0];

    // setup the language infrastructure
    final SimLangLang lang = new SimLangLang();

    // parse the model and create the AST representation
    final ASTSimLangCompilationUnit ast = parse(model);
    Log.info(model + " parsed successfully!", SimLangTool.class.getName());

    // setup the symbol table
    Scope modelTopScope = createSymbolTable(lang, ast);

    //Run Context-Conditions
    Log.info("Running CoCos...", SimLangTool.class.getName());
    checkDefaultCoCos(ast);
    Log.info("Finished CoCos.", SimLangTool.class.getName());

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
      Log.warn("Model could not be parsed.");
    }
    catch (RecognitionException | IOException e) {
      Log.warn("Failed to parse " + model, e);
    }
    return null;
  }

  public static void checkDefaultCoCos(ASTSimLangCompilationUnit ast) {
    SimLangCoCoChecker defaultCoCos = new SimLangCoCoChecker();

    Log.info("Data CoCo...",SimLangTool.class.getName());
    //DataStructure Checkers --Need to runs first
    defaultCoCos.addCoCo(new NoInfRangeChecker());
    defaultCoCos.addCoCo(new CoordinateChecker());

    Log.info("Simulation CoCo...",SimLangTool.class.getName());
    //Simulation CoCos
    defaultCoCos.addCoCo(new RequiredSimulationEntries());
    defaultCoCos.addCoCo(new NoMultipleSimulationEntries());
    defaultCoCos.addCoCo(new SimulationRenderFrequencyChecker());
    defaultCoCos.addCoCo(new SimulationLoopFrequencyChecker());
    defaultCoCos.addCoCo(new SimulationDurationChecker());
    defaultCoCos.addCoCo(new MapNameChecker());
    defaultCoCos.addCoCo(new TimeChecker());
    defaultCoCos.addCoCo(new TimeoutChecker());
    defaultCoCos.addCoCo(new GravityChecker());
    defaultCoCos.addCoCo(new PedestrianDensityChecker());

    Log.info("Weather CoCo...",SimLangTool.class.getName());
    //Weather CoCos
    defaultCoCos.addCoCo(new NoMultipleWeatherObjEntries());
    defaultCoCos.addCoCo(new TemperatureChecker());
    defaultCoCos.addCoCo(new PressureChecker());
    defaultCoCos.addCoCo(new HumidityChecker());
    defaultCoCos.addCoCo(new SightChecker());
    defaultCoCos.addCoCo(new PrecipitationAmountChecker());
    defaultCoCos.addCoCo(new WindStrengthChecker());
    defaultCoCos.addCoCo(new WindDirectionChecker());

    Log.info("Channel CoCo...", SimLangTool.class.getName());
    //Communication CoCos
    defaultCoCos.addCoCo(new NoDuplicateChannelNames());
    defaultCoCos.addCoCo(new OneChannelEntryEach());
    defaultCoCos.addCoCo(new TransferrateChecker());
    defaultCoCos.addCoCo(new LatencyChecker());
    defaultCoCos.addCoCo(new OutageChecker());
    defaultCoCos.addCoCo(new AreaChecker());

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
    // resolverConfiguration.addTopScopeResolvers(lang.getResolvers());
    resolverConfiguration.addDefaultFilters(lang.getResolvingFilters());

    GlobalScope globalScope = new GlobalScope(new ModelPath(), lang, resolverConfiguration);

    Optional<SimLangSymbolTableCreator> symbolTable = lang.getSymbolTableCreator(resolverConfiguration, globalScope);
    return symbolTable.get().createFromAST(ast);
  }

  /*
  public static Scope createSymbolTable(ModelingLanguageFamily lang, ASTSimLangCompilationUnit astSim, ASTCar astCar) {

    return null;
  }
  */

  public static SimLangContainer parseIntoContainer(String model) {
    final SimLangLang lang = new SimLangLang();
    final ASTSimLangCompilationUnit ast = parse(model);
    final Scope modelTopScope = createSymbolTable(lang, ast);
    checkDefaultCoCos(ast);
    return new SimLangContainer(modelTopScope, String.join(".",ast.getPackageList()), ast.getSimulation().getName());
  }
  /*
  public static SimLangContainer parseIntoContainer(String simModel, String carModel) {
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
    return new SimLangContainer(modelTopScope);
  }
  */
}
