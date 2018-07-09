/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Collection;
import java.util.NoSuchElementException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.InstancingRegister;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

/**
 * Created by Michael von Wenckstern on 30.05.2016.
 *
 * @author Michael von Wenckstern
 */
public class EmbeddedMontiArcModelLoader extends EmbeddedMontiArcModelLoaderTOP {


  public EmbeddedMontiArcModelLoader() {
    super(new EmbeddedMontiArcLanguage());
  }

  public EmbeddedMontiArcModelLoader(EmbeddedMontiArcLanguage language) {
    super(language);
  }


  @Override
  public Collection<ASTEMACompilationUnit> loadModelsIntoScope(final String qualifiedModelName,
      final ModelPath modelPath, final MutableScope enclosingScope,
      final ResolvingConfiguration ResolvingConfiguration) {

    final Collection<ASTEMACompilationUnit> asts = loadModels(qualifiedModelName, modelPath);

    for (ASTEMACompilationUnit ast : asts) {
      createSymbolTableFromAST(ast, qualifiedModelName, enclosingScope, ResolvingConfiguration);
    }

    return asts;
  }

  /**
   *  Use this Method to load the top Component
   * @param mainTxt Path to main.txt
   * @return
   */
  public EMAComponentSymbol loadComponentFromMainTxt(String mainTxt) {
    Scope scope = createSymTabFromMainTxt(mainTxt);
    String mainComponent = parseMainComponent(mainTxt);

    EMAComponentSymbol emaComponentSymbol = scope.<EMAComponentSymbol>resolve(mainComponent, EMAComponentSymbol.KIND).orElse(null);

    if(emaComponentSymbol == null)
      Log.error("Could not resolve mainComponent form mainTxt: " + mainComponent);

    return emaComponentSymbol;
  }

  /**
   *  Use this Method to create the symTab for a model
   * @param mainTxt Path to main.txt
   * @return
   */
  public Scope createSymTabFromMainTxt(String mainTxt) {
    String mainComponent = parseMainComponent(mainTxt);
    String mainInstantiation = parseMainInstantiation(mainTxt);
    ModelPath modelPath = parseModelPath(mainTxt);

    Scope scope = createSymTab(modelPath, mainComponent, mainInstantiation);

    EMAComponentSymbol emaComponentSymbol = scope.<EMAComponentSymbol>resolve(mainComponent, EMAComponentSymbol.KIND).orElse(null);

    if(emaComponentSymbol == null)
      Log.error("Could not resolve mainComponent form mainTxt: " + mainComponent);

    return scope;
  }

  protected Scope createSymTab(ModelPath modelPath, String mainComponent, String mainInstantiation) {
    ModelingLanguageFamily fam = new ModelingLanguageFamily();
    fam.addModelingLanguage(new EmbeddedMontiArcLanguage());
    fam.addModelingLanguage(new StreamLanguage());
    fam.addModelingLanguage(new StructLanguage());

    GlobalScope scope = new GlobalScope(modelPath, fam);

    de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);

    LogConfig.init();
    InstancingRegister.reset();
    InstancingRegister.mainComponent = mainComponent;
    InstancingRegister.mainInstantiation = mainInstantiation;

    return scope;
  }

  private static String parseMainComponent(String mainTxt) {
    String mainComponent = "defaultComponent";
    try {
      Pattern p = Pattern.compile("Main-Component-Instantiation:\\s+(\\w+(?:\\.\\w+)*)\\s+(\\w+);");
      Stream<String> lines = Files.lines(Paths.get(mainTxt));
      mainComponent = lines.map(p::matcher)
              .filter(Matcher::matches)
              .findFirst().get().group(1);
    }
    catch (IOException e) {
      Log.error("Could not read: " + mainTxt);
    }
    catch (NoSuchElementException e) {
      Log.error("Could not parse main component.");
    }
    return mainComponent;
  }

  private static String parseMainInstantiation(String mainTxt) {
    String mainInstantiation = "defaultInstantiation";
    try {
      Pattern p = Pattern.compile("Main-Component-Instantiation:\\s+(\\w+(?:\\.\\w+)*)\\s+(\\w+);");
      Stream<String> lines = Files.lines(Paths.get(mainTxt));
      mainInstantiation = lines.map(p::matcher)
              .filter(Matcher::matches)
              .findFirst().get().group(2);
    }
    catch (IOException e) {
      Log.error("Could not read: " + mainTxt);
    }
    catch (NoSuchElementException e) {
      Log.error("Could not parse main instantiation.");
    }
    return mainInstantiation;
  }

  private static ModelPath parseModelPath(String mainTxt) {
    ModelPath modelPath = new ModelPath();
    try {
      Pattern mpLine = Pattern.compile("Model-Paths:\\s+\\[(.*)\\]");
      Stream<String> lines = Files.lines(Paths.get(mainTxt));
      String mpString = lines.map(mpLine::matcher)
              .filter(Matcher::matches)
              .findFirst().get().group(1);

      Matcher mpEntry = Pattern.compile("((?:\\.|\\.\\.|/|\\w+)+)").matcher(mpString);

      Path mainTxtDir = Paths.get(mainTxt).getParent();
      while(mpEntry.find()) {
        modelPath.addEntry(Paths.get(mainTxtDir.toString() + File.separator + mpEntry.group()));
      }
    }
    catch (IOException e) {
      Log.error("Could not read: " + mainTxt);
    }
    catch (NoSuchElementException e) {
      Log.error("Could not parse main instantiation.");
    }
    return modelPath;
  }
}
