/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.emam2wasm.web.HtmlGeneratorFactory;
import de.monticore.lang.monticar.emam2wasm.web.HtmlInterfaceNameProvider;
import de.monticore.lang.monticar.emam2wasm.web.JsGeneratorFactory;
import de.monticore.lang.monticar.emam2wasm.web.JsWrapperNameProvider;
import de.monticore.lang.monticar.emam2wasm.web.WebStep;
import de.monticore.lang.monticar.freemarker.TemplateFactory;
import de.monticore.lang.monticar.generator.FileContentAssert;
import de.monticore.lang.monticar.junit.TemporaryDirectoryExtension;
import de.monticore.lang.monticar.resolver.Resolver;
import de.monticore.lang.monticar.resolver.ResolverFactory;
import de.monticore.lang.monticar.util.TextFile;
import freemarker.cache.FileTemplateLoader;
import freemarker.cache.MultiTemplateLoader;
import freemarker.cache.TemplateLoader;
import freemarker.template.Template;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

class WebStepIT {

  private static final Path JS_TEMPLATE_DIR = Paths.get("src/main/resources/ftl/js");
  private static final String JS_TEMPLATE = "js.ftl";
  private static final Path HTML_TEMPLATE_DIR = Paths.get("src/main/resources/ftl/html");
  private static final String HTML_TEMPLATE = "html.ftl";
  private static final Path RESSOURCE_DIR = Paths
      .get("src/integration-test/resources/webstep");
  private static final Path EXPECTED_DIR = RESSOURCE_DIR.resolve("expected");
  private static final Path RESOLVING_BASE_DIR = RESSOURCE_DIR;
  private static final String SOME_MODEL = "models.multiplePorts";
  private static final String WASM_FILE_NAME = "multiplePorts.js";
  private static final JsWrapperNameProvider WRAPPER_NAME_PROVIDER = new JsWrapperNameProvider();
  private static final HtmlInterfaceNameProvider INTERFACE_NAME_PROVIDER = new HtmlInterfaceNameProvider();

  private Template jsTemplate;
  private Template htmlTemplate;
  private ExpandedComponentInstanceSymbol model;

  @BeforeEach
  void setUp() throws IOException {
    TemplateFactory templateFactory = setUpTemplateFactory();
    jsTemplate = templateFactory.getTemplate(JS_TEMPLATE);
    htmlTemplate = templateFactory.getTemplate(HTML_TEMPLATE);

    Resolver resolver = setUpResolver();
    model = resolver.getExpandedComponentInstanceSymbol(SOME_MODEL);
  }

  @Test
  @ExtendWith(TemporaryDirectoryExtension.class)
  void shouldGenerateHtml(Path dir) {
    WebStep webStep = new WebStep(new JsGeneratorFactory(jsTemplate),
        new HtmlGeneratorFactory(htmlTemplate), dir, WRAPPER_NAME_PROVIDER,
        INTERFACE_NAME_PROVIDER);

    webStep.generateWebInterface(model, dir.resolve(WASM_FILE_NAME));

    TextFile actualHtml = new TextFile(dir.resolve(INTERFACE_NAME_PROVIDER.getFilePath(model)));
    TextFile expectedHtml = new TextFile(EXPECTED_DIR.resolve("multiplePorts.html"));
    FileContentAssert.assertThat(actualHtml.read()).isEqualToFileContent(expectedHtml);
  }

  @Test
  @ExtendWith(TemporaryDirectoryExtension.class)
  void shouldGenerateJs(Path dir) {
    WebStep webStep = new WebStep(new JsGeneratorFactory(jsTemplate),
        new HtmlGeneratorFactory(htmlTemplate), dir, WRAPPER_NAME_PROVIDER,
        INTERFACE_NAME_PROVIDER);

    webStep.generateWebInterface(model, dir.resolve(WASM_FILE_NAME));

    TextFile actualJs = new TextFile(dir.resolve(WRAPPER_NAME_PROVIDER.getFilePath(model)));
    TextFile expectedJs = new TextFile(EXPECTED_DIR.resolve("multiplePorts_wrapper.js"));
    FileContentAssert.assertThat(actualJs.read()).isEqualToFileContent(expectedJs);
  }

  private TemplateFactory setUpTemplateFactory() throws IOException {
    FileTemplateLoader jsFtl = new FileTemplateLoader(JS_TEMPLATE_DIR.toFile());
    FileTemplateLoader htmlFtl = new FileTemplateLoader(HTML_TEMPLATE_DIR.toFile());
    MultiTemplateLoader mtl = new MultiTemplateLoader(new TemplateLoader[]{jsFtl, htmlFtl});
    return new TemplateFactory(mtl);
  }

  private Resolver setUpResolver() {
    ResolverFactory resolverFactory = new ResolverFactory(RESOLVING_BASE_DIR);
    return resolverFactory.get();
  }
}
