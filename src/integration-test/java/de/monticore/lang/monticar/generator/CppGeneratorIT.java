/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import static de.monticore.lang.monticar.generator.FileContentAssert.assertThat;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.freemarker.SimpleTemplatePrinter;
import de.monticore.lang.monticar.freemarker.TemplateFactory;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.generator.cpp.CppGenerator;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import freemarker.cache.FileTemplateLoader;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CppGeneratorIT {

  private static final Path TEMPLATE_DIR = Paths.get("src/main/resources/ftl/cpp");
  private static final String CPP_TEMPLATE = "cpp.ftl";

  private static final Path RESOLVING_BASE_DIR = Paths
      .get("src/integration-test/resources/cppfilegeneration");
  private static final String MODEL_WITH_MATRIX_FULL_NAME = "models.matrixPort";
  private static final String MODEL_WITH_MATRIX_ARRAY_FULL_NAME = "models.matrixArrayPort";
  private static final String MODEL_WITH_SCALAR_FULL_NAME = "models.scalarPort";
  private static final String MODEL_WITH_ARRAY_FULL_NAME = "models.arrayPort";

  private static final Path CPP_FILE_MATRIX = RESOLVING_BASE_DIR
      .resolve("expected/models_matrixPort.cpp");
  private static final Path CPP_FILE_MATRIX_ARRAY = RESOLVING_BASE_DIR
      .resolve("expected/models_matrixArrayPort.cpp");
  private static final Path CPP_FILE_SCALAR = RESOLVING_BASE_DIR
      .resolve("expected/models_scalarPort.cpp");
  private static final Path CPP_FILE_ARRAY = RESOLVING_BASE_DIR
      .resolve("expected/models_arrayPort.cpp");

  private CppGenerator generator;
  private Resolver resolver;
  private Writer writer;

  @BeforeEach
  void setUp() throws IOException {
    FileTemplateLoader ftl = new FileTemplateLoader(TEMPLATE_DIR.toFile());
    TemplateFactory templateFactory = new TemplateFactory(ftl);
    Template template = templateFactory.getTemplate(CPP_TEMPLATE);
    writer = new StringWriter();
    TemplateProcessor templateProcessor = new SimpleTemplatePrinter(template, writer);
    generator = new CppGenerator(templateProcessor);

    ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
    resolver = resolverFactory.get();
  }

  @Test
  void withMatrixPort() throws IOException, TemplateException {
    ExpandedComponentInstanceSymbol matrixModel = resolver
        .getExpandedComponentInstanceSymbol(MODEL_WITH_MATRIX_FULL_NAME);

    generator.generate(matrixModel);

    assertThat(writer.toString()).isEqualToFileContent(CPP_FILE_MATRIX);
  }

  @Test
  void withMatrixArrayPort() throws IOException, TemplateException {
    ExpandedComponentInstanceSymbol matrixModel = resolver
        .getExpandedComponentInstanceSymbol(MODEL_WITH_MATRIX_ARRAY_FULL_NAME);

    generator.generate(matrixModel);

    assertThat(writer.toString()).isEqualToFileContent(CPP_FILE_MATRIX_ARRAY);
  }

  @Test
  void withScalarPort() throws IOException, TemplateException {
    ExpandedComponentInstanceSymbol matrixModel = resolver
        .getExpandedComponentInstanceSymbol(MODEL_WITH_SCALAR_FULL_NAME);

    generator.generate(matrixModel);

    assertThat(writer.toString()).isEqualToFileContent(CPP_FILE_SCALAR);
  }

  @Test
  void withArrayPort() throws IOException, TemplateException {
    ExpandedComponentInstanceSymbol matrixModel = resolver
        .getExpandedComponentInstanceSymbol(MODEL_WITH_ARRAY_FULL_NAME);

    generator.generate(matrixModel);

    assertThat(writer.toString()).isEqualToFileContent(CPP_FILE_ARRAY);
  }
}
