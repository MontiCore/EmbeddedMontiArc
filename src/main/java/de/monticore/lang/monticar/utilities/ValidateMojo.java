/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;

import de.monticore.antlr4.MCConcreteParser;
import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.emadl.generator.EMADLGenerator;
import de.monticore.lang.monticar.enumlang._ast.ASTEnumLangCompilationUnit;
import de.monticore.lang.monticar.enumlang._symboltable.EnumDeclarationSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.streamunits._ast.ASTStreamUnitsCompilationUnit;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.utilities.utils.SearchFiles;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.apache.commons.io.FileUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.apache.maven.plugins.annotations.Mojo;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

//import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;

/**
 * Runs CoCos test on all components in pathMain and pathTest
 * Generates c++ code for all components which have a stream test
 */
@Mojo(name = "validate")
public class ValidateMojo extends StreamTestMojoBase {

  @Override
  protected void mainExecution() throws MojoExecutionException {
    Log.info("StreamTestGeneratorMojo", "StreamTestGeneratorMojo");

    checkCocosOfInputFiles();
  }

  protected void checkCocosOfInputFiles() throws MojoExecutionException {
    Map<String,MCConcreteParser> parser = getParser();
    Scope scope = getScope();

    logInfo("Cocos Check:");

    List<File> ff = SearchFiles.searchFiles(this.pathMain, "emam", "struct", "enum","emadl");
    Map<String, File> files = SearchFiles.searchFilesMap(this.pathMain, "emam", "struct", "enum","emadl");
    files.putAll(SearchFiles.searchFilesMap(this.pathTest, "emam", "stream","emadl"));

    for (Map.Entry<String,File> f:files.entrySet()) {
      String ending = f.getKey().substring(f.getKey().lastIndexOf(".") + 1);
      if (!parser.keySet().contains(ending)) {
        throw new MojoExecutionException("No parser for ." + ending + " files");
        //errors.add(f.getKey()+" (error: no parser for file)");
      }
      MCConcreteParser mccp = parser.get(ending);
      logInfo(" - "+f.getKey());
      try {
        Optional<? extends ASTNode> node = mccp.parse(f.getValue().getAbsolutePath());
        if(!node.isPresent()){
          logError("   -> Could not parse: "+f.getKey());
        }else {
          boolean resolved = false;
          String modelName;
          if(ending.equalsIgnoreCase("emam")) {
            ASTEMACompilationUnit ast = (ASTEMACompilationUnit) node.get();

            modelName = modelNameCalculator(f.getValue(),"emam", ast.getPackageList());
            Optional<EMAComponentSymbol> comp = scope.<EMAComponentSymbol>resolve(modelName, EMAComponentSymbol.KIND);
            resolved = comp.isPresent();
          }else if(ending.equalsIgnoreCase("stream")) {
            ASTStreamUnitsCompilationUnit ast = (ASTStreamUnitsCompilationUnit) node.get();

            modelName = modelNameCalculator(f.getValue(), "stream", ast.getPackageList());
            Optional<ComponentStreamUnitsSymbol> comp = scope.<ComponentStreamUnitsSymbol>resolve(modelName, ComponentStreamUnitsSymbol.KIND);
            resolved = comp.isPresent();
          }else if(ending.equalsIgnoreCase("struct")) {
            ASTStructCompilationUnit ast = (ASTStructCompilationUnit) node.get();
            modelName = modelNameCalculator(f.getValue(), "struct", ast.getPackageList());
            Optional<StructSymbol> structSym = scope.resolve(modelName, StructSymbol.KIND);
            resolved = structSym.isPresent();
          }else if(ending.equalsIgnoreCase("enum")){
            ASTEnumLangCompilationUnit ast = (ASTEnumLangCompilationUnit)node.get();
            modelName = modelNameCalculator(f.getValue(), "enum", ast.getPackageList());
            Optional<EnumDeclarationSymbol> enumSym = scope.resolve(modelName, EnumDeclarationSymbol.KIND);
            resolved = enumSym.isPresent();
          }else if (ending.equalsIgnoreCase("emadl")){
            ASTEMACompilationUnit ast = (ASTEMACompilationUnit) node.get();

            modelName = modelNameCalculator(f.getValue(),"emadl", ast.getPackageList());
            Optional<EMAComponentSymbol> comp = scope.<EMAComponentSymbol>resolve(modelName, EMAComponentSymbol.KIND);
            resolved = comp.isPresent();
          }else{
            //TODO:
            logWarn("   -> No resolving for "+ending+" implemented at the moment.");
          }

          if(resolved){
            logInfo("   -> parsed & resolved");
          }else{
            logError("Could not resolve "+f.getKey());
            throw new MojoExecutionException("Some files are invalid");
          }

        }
      } catch (IOException e) {
        e.printStackTrace();
      }

    }
  }

  protected String modelNameCalculator(File f, String ending,  List<String> packages){
    String packageName = "";
    if(!packages.isEmpty()) {
      packageName = Joiners.DOT.join(packages) + ".";
    }
    return packageName+f.getName().replace("."+ending, "");
  }
}
