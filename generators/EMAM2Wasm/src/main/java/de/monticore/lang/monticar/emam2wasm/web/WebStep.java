/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.html.HtmlGenerator;
import de.monticore.lang.monticar.generator.js.JsGenerator;
import freemarker.template.TemplateException;
import java.io.IOException;
import java.nio.file.Path;
import org.apache.commons.io.FilenameUtils;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

/**
 * This class generates a web interface for a WebAssembly module implementing
 * the behavior of a EmbeddedMontiArc model.
 *
 * @see de.monticore.lang.monticar.emam2wasm.wasm.WasmStep
 */
@Component
public class WebStep {

  private JsGeneratorFactory jsGeneratorFactory;
  private HtmlGeneratorFactory htmlGeneratorFactory;
  private Path webDir;
  private WrapperNameProvider wrapperNameProvider;
  private InterfaceNameProvider interfaceNameProvider;

  @Autowired
  public WebStep(JsGeneratorFactory jsGeneratorFactory, HtmlGeneratorFactory htmlGeneratorFactory,
      Path webDir, WrapperNameProvider wrapperNameProvider,
      InterfaceNameProvider interfaceNameProvider) {
    this.jsGeneratorFactory = requiresNotNull(jsGeneratorFactory);
    this.htmlGeneratorFactory = requiresNotNull(htmlGeneratorFactory);
    this.webDir = requiresNotNull(webDir);
    this.wrapperNameProvider = requiresNotNull(wrapperNameProvider);
    this.interfaceNameProvider = requiresNotNull(interfaceNameProvider);
  }

  private static Path relative(Path base, Path other) {
    return normalize(base).relativize(normalize(other)).normalize();
  }

  private static Path normalize(Path path) {
    return path.toAbsolutePath().normalize();
  }

  private static String getBaseFileName(Path file) {
    return FilenameUtils.getBaseName(file.toString());
  }

  /**
   * Generates a web interface for the supplied model. This interface will
   * consist of a Javascript wrapper for a previously generated WebAssembly
   * module and an HTML interface. This HTML interface will provide input
   * fields for ingoing ports of the model and text labels for outgoing ports.
   *
   * @param model model to generate a web interface for
   * @param wasmFile WebAssembly file that implements the actions of the model
   * and is used by the web interface to calculate the model's output for given
   * input.
   * @return path to the wrapper file
   * @throws WebInterfaceGeneratorException if the web files generation fails
   */
  public Path generateWebInterface(ExpandedComponentInstanceSymbol model, Path wasmFile) {
    requiresNotNull(model);
    requiresNotNull(wasmFile);

    Path wrapperPath = webDir.resolve(wrapperNameProvider.getFilePath(model));
    JsGenerator jsGenerator = jsGeneratorFactory.getJsGenerator(wrapperPath);

    Path htmlFile = webDir.resolve(interfaceNameProvider.getFilePath(model));
    String wasmName = getBaseFileName(relative(webDir, wasmFile));
    HtmlGenerator htmlGenerator = htmlGeneratorFactory
        .getHtmlGenerator(htmlFile, cmp -> wasmName, cmp -> wrapperNameProvider.getName(cmp));

    try {
      jsGenerator.generate(model);
      htmlGenerator.generate(model);
    } catch (IOException | TemplateException e) {
      throw new WebInterfaceGeneratorException(e);
    }

    return wrapperPath;
  }
}
