/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.fail;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.adapter.CppFileGenerator;
import de.monticore.lang.monticar.adapter.CppGeneratorAdapter;
import de.monticore.lang.monticar.adapter.EmamCppCompiler;
import de.monticore.lang.monticar.adapter.GeneratorCppWrapper;
import de.monticore.lang.monticar.emam2wasm.cpp.CppCompiler;
import de.monticore.lang.monticar.emam2wasm.cpp.CppMainNameProvider;
import de.monticore.lang.monticar.emam2wasm.cpp.CppNameProvider;
import de.monticore.lang.monticar.emam2wasm.cpp.CppStep;
import de.monticore.lang.monticar.freemarker.TemplateFactory;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.SymTabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import freemarker.cache.FileTemplateLoader;
import freemarker.template.Template;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class CppStepIT {

  private static final Path MODEL_PATH = Paths.get("src/integration-test/resources/cppstep");
  private static final Path EXPECTED = MODEL_PATH.resolve("expected");
  private static final Path TEMPLATE_DIR = Paths.get("src/main/resources/ftl/cpp");
  private static final String CPP_TEMPLATE_NAME = "cpp.ftl";
  private static final String SOME_MODEL = "models.addToArray";
  private TaggingResolver symtab;
  private Resolver resolver;

  @BeforeEach
  void setUp() {
    SymTabCreator symTabCreator = new SymTabCreator(MODEL_PATH);
    symtab = symTabCreator.createSymTabAndTaggingResolver();
    resolver = new Resolver(symtab);
  }

  @Test
  @ExtendWith(TemporaryDirectoryExtension.class)
  void shouldGenerate(Path dir) throws IOException {
    ExpandedComponentInstanceSymbol model = resolver
        .getExpandedComponentInstanceSymbol(SOME_MODEL);
    CppStep<ExpandedComponentInstanceSymbol> cppStep = new CppStep<>(
        cppCompiler(), dir, cppNameProvider());

    cppStep.compileToCpp(model);

    assertFolderContentsAreEqual(dir, EXPECTED);
  }

  private CppCompiler<ExpandedComponentInstanceSymbol> cppCompiler() throws IOException {
    GeneratorCppWrapper generatorCppWrapper = new GeneratorCppWrapper(new GeneratorCPP(), symtab,
        MODEL_PATH);
    CppFileGenerator cppFileGenerator = new CppGeneratorAdapter(cppTemplate());
    return new EmamCppCompiler(generatorCppWrapper, cppFileGenerator);
  }

  private CppNameProvider cppNameProvider() {
    return new CppMainNameProvider();
  }

  private Template cppTemplate() throws IOException {
    FileTemplateLoader ftl = new FileTemplateLoader(TEMPLATE_DIR.toFile());
    TemplateFactory templateFactory = new TemplateFactory(ftl);
    return templateFactory.getTemplate(CPP_TEMPLATE_NAME);
  }

  private void assertFolderContentsAreEqual(Path actual, Path expected) throws IOException {
    List<Path> actualFiles = files(actual);
    List<Path> expectedFiles = files(expected);

    for (Path actualFile : actualFiles) {
      for (Path expectedFile : expectedFiles) {
        if (actualFile.getFileName().equals(expectedFile.getFileName())) {
          if (Files.isRegularFile(actualFile) && Files.isRegularFile(expectedFile)) {
            assertThat(actualFile).hasSameContentAs(expectedFile);
          } else {
            fail("No matching file was found for " + actualFile.toString());
          }
        }
      }
    }
  }

  private List<Path> files(Path dir) throws IOException {
    try (Stream<Path> fileStream = Files.walk(dir)) {
      return fileStream.collect(Collectors.toList());
    }
  }
}
