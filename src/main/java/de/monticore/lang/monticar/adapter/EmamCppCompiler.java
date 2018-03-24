package de.monticore.lang.monticar.adapter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emam2wasm.cpp.CppCompiler;
import de.monticore.lang.monticar.emam2wasm.cpp.CppCompilerException;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

@Component
public class EmamCppCompiler implements CppCompiler<ExpandedComponentInstanceSymbol> {

  private final GeneratorCppWrapper modelCompiler;
  private final CppFileGenerator cppFileGenerator;
  private final TaggingResolver symtab;

  @Autowired
  public EmamCppCompiler(GeneratorCppWrapper modelCompiler, CppFileGenerator cppFileGenerator,
      TaggingResolver symtab) {
    this.modelCompiler = modelCompiler;
    this.cppFileGenerator = cppFileGenerator;
    this.symtab = symtab;
  }

  @Override
  public Path compile(ExpandedComponentInstanceSymbol symbol, Path outDir, String mainFileName) {
    try {
      Path mainFile = outDir.resolve(mainFileName);
      compileModel(symtab, symbol, outDir);
      return generateCppFile(symbol, mainFile);
    } catch (IOException | TemplateException e) {
      throw new CppCompilerException(e);
    }
  }

  private void compileModel(TaggingResolver symtab, ExpandedComponentInstanceSymbol model,
      Path outDir) throws IOException {
    modelCompiler.setTargetPath(outDir);
    modelCompiler.generateFiles(model, symtab);
  }

  private Path generateCppFile(ExpandedComponentInstanceSymbol model, Path mainFile)
      throws IOException, TemplateException {
    return cppFileGenerator.generate(model, mainFile);
  }
}
