/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import static de.monticore.lang.monticar.generator.FileContentAssert.assertThat;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.freemarker.SimpleTemplatePrinter;
import de.monticore.lang.monticar.freemarker.TemplateFactory;
import de.monticore.lang.monticar.freemarker.TemplateProcessor;
import de.monticore.lang.monticar.generator.html.HtmlGenerator;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import de.monticore.symboltable.Symbol;
import freemarker.cache.FileTemplateLoader;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.io.StringWriter;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Function;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class HtmlGeneratorIT {

  private static final Path RESSOURCE_DIR = Paths
      .get("src/integration-test/resources/htmlfilegeneration");
  private static final Path EXPECTED_DIR = RESSOURCE_DIR.resolve("expected");
  private static final Path TEMPLATE_DIR = Paths.get("src/main/resources/ftl/html");
  private static final String HTML_TEMPLATE = "html.ftl";

  private static final Path RESOLVING_BASE_DIR = RESSOURCE_DIR;
  private static final String PACKAGE = "models";
  private static final String HTML_FILE_EXTENSION = "html";
  private static final String WRAPPER_SUFFIX = "wrapper";


  private StringWriter writer;
  private HtmlGenerator generator;
  private Resolver resolver;

  @BeforeEach
  void setUp() throws IOException {
    FileTemplateLoader ftl = new FileTemplateLoader(TEMPLATE_DIR.toFile());
    TemplateFactory templateFactory = new TemplateFactory(ftl);
    Template template = templateFactory.getTemplate(HTML_TEMPLATE);
    writer = new StringWriter();
    TemplateProcessor templateProcessor = new SimpleTemplatePrinter(template, writer);
    generator = new HtmlGenerator(templateProcessor, wasmNamingFunction(), wrapperNamingFunction());
    ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
    resolver = resolverFactory.get();
  }

  @ParameterizedTest
  @ValueSource(strings = {
      "noPorts",
      "scalar",
      "array",
      "matrix",
      "multiplePorts"
  })
  void testModel(String name) throws IOException, TemplateException {
    ExpandedComponentInstanceSymbol model = resolver
        .getExpandedComponentInstanceSymbol(PACKAGE + "." + name);

    generator.generate(model);

    assertThat(writer.toString())
        .isEqualToFileContent(EXPECTED_DIR.resolve(name + "." + HTML_FILE_EXTENSION));
  }

  private Function<ExpandedComponentInstanceSymbol, String> wasmNamingFunction() {
    return Symbol::getName;
  }

  private Function<ExpandedComponentInstanceSymbol, String> wrapperNamingFunction() {
    return model -> model.getName() + "_" + WRAPPER_SUFFIX;
  }

}
