package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.monticar.utilities.models.DatasetToStore;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class ModelArtifactCreator {

  public static void createArtifact(DatasetToStore modelToStore) throws IOException {
    EMADLParser parser = new EMADLParser();
    Optional<ASTEMACompilationUnit> node = parser.parse(modelToStore.getPath().getAbsolutePath());

    if (!node.isPresent()) {
      System.out.println("NOT PRESENT");
      return;
    }

    ASTEMACompilationUnit ast = node.get();
    String modelName = createModelName(ast.getComponent().getName(), ast.getPackageList());

    Scope scope = createScope(modelToStore.getPath());
    Optional<EMAComponentSymbol> comp = scope.<EMAComponentSymbol>resolve(modelName, EMAComponentSymbol.KIND);
  }

  private static String createModelName(String name, List<String> packageList) {
    String packageName = packageList.isEmpty() ? "" : String.join(".", packageList) + ".";
    return packageName + name;
  }

  private static Scope createScope(File modelPath) {
    ModelingLanguageFamily modelingLanguageFamily = new ModelingLanguageFamily();
    modelingLanguageFamily.addModelingLanguage(new EMADLLanguage());

    final ModelPath mp = new ModelPath(modelPath.toPath());
    GlobalScope globalScope = new GlobalScope(new ModelPath(), new ModelingLanguageFamily());
    return globalScope;
  }

}
