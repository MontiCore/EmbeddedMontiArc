/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emam2wasm.cpp.CppCompiler;
import de.monticore.lang.monticar.emam2wasm.cpp.CppCompilerException;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

@Component
public class EmamCppCompiler implements CppCompiler<ExpandedComponentInstanceSymbol> {

  private final GeneratorCppWrapper modelCompiler;
  private final CppFileGenerator cppFileGenerator;

  @Autowired
  public EmamCppCompiler(GeneratorCppWrapper modelCompiler, CppFileGenerator cppFileGenerator) {
    this.modelCompiler = modelCompiler;
    this.cppFileGenerator = cppFileGenerator;
  }

  @Override
  public Path compile(ExpandedComponentInstanceSymbol symbol, Path outDir, String mainFileName) {
    try {
      Path mainFile = outDir.resolve(mainFileName);
      compileModel(symbol, outDir);
      return generateCppFile(symbol, mainFile);
    } catch (IOException | TemplateException e) {
      throw new CppCompilerException(e);
    }
  }

  private void compileModel(ExpandedComponentInstanceSymbol model,
      Path outDir) throws IOException {
    modelCompiler.setTargetPath(outDir);
    modelCompiler.generateFiles(model);
  }

  private Path generateCppFile(ExpandedComponentInstanceSymbol model, Path mainFile)
      throws IOException, TemplateException {
    return cppFileGenerator.generate(model, mainFile);
  }
}
