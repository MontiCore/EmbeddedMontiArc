/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import static de.monticore.lang.monticar.generator.FileContentAssert.assertThat;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.freemarker.SimpleTemplatePrinter;
import de.monticore.lang.monticar.freemarker.TemplateFactory;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.generator.js.JsGenerator;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import freemarker.cache.FileTemplateLoader;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class JsGeneratorIT {

  private static final Path RESSOURCE_DIR = Paths
      .get("src/integration-test/resources/jsfilegeneration");
  private static final Path EXPECTED_DIR = RESSOURCE_DIR.resolve("expected");
  private static final Path TEMPLATE_DIR = Paths.get("src/main/resources/ftl/js");
  private static final String JS_TEMPLATE = "js.ftl";

  private static final Path RESOLVING_BASE_DIR = RESSOURCE_DIR;
  private static final String PACKAGE = "models";
  private static final String JS_FILE_EXTENSION = "js";


  private StringWriter writer;
  private JsGenerator generator;
  private Resolver resolver;

  @BeforeEach
  void setUp() throws IOException {
    FileTemplateLoader ftl = new FileTemplateLoader(TEMPLATE_DIR.toFile());
    TemplateFactory templateFactory = new TemplateFactory(ftl);
    Template template = templateFactory.getTemplate(JS_TEMPLATE);
    writer = new StringWriter();
    TemplateProcessor templateProcessor = new SimpleTemplatePrinter(template, writer);
    generator = new JsGenerator(templateProcessor);

    ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
    resolver = resolverFactory.get();
  }

  @ParameterizedTest
  @ValueSource(strings = {
      "noPorts", "caretSymbol", "thousandsDimension",
      "scalar_NoRangeNoUnit", "scalar_RangeNoUnit", "scalar_RangeUnit",
      "array_NoRangeNoUnit", "array_RangeNoUnit", "array_RangeUnit",
      "rowVector_NoRangeNoUnit", "rowVector_RangeNoUnit", "rowVector_RangeUnit",
      "columnVector_NoRangeNoUnit", "columnVector_RangeNoUnit", "columnVector_RangeUnit",
      "matrix_NoRangeNoUnit", "matrix_RangeNoUnit", "matrix_RangeUnit",
      "matrixArray_NoRangeNoUnit", "matrixArray_RangeNoUnit", "matrixArray_RangeUnit"
  })
  void testModel(String name) throws IOException, TemplateException {
    ExpandedComponentInstanceSymbol model = resolver
        .getExpandedComponentInstanceSymbol(PACKAGE + "." + name);

    generator.generate(model);

    assertThat(writer.toString())
        .isEqualToFileContent(EXPECTED_DIR.resolve(name + "." + JS_FILE_EXTENSION));
  }
}
