/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emam2wasm.web;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.freemarker.FileTemplatePrinter;
import de.monticore.lang.monticar.generator.html.HtmlGenerator;
import freemarker.template.Template;
import java.nio.file.Path;
import java.util.function.Function;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Component;

/**
 * Factory for {@link HtmlGenerator}.
 */
@Component
public class HtmlGeneratorFactory {

  private Template template;

  /**
   * Creates a new factory
   *
   * @param template the template that the {@link HtmlGenerator} will use to
   * produce HTML code
   */
  @Autowired
  public HtmlGeneratorFactory(@Qualifier("htmlTemplate") Template template) {
    this.template = template;
  }

  /**
   * Produces a new {@link HtmlGenerator}.
   *
   * @param htmlFile path to the file that the generator will write to
   * @param wasmNamingFunction provides the name of the WebAssembly module
   * that the created generator will generate an interface for
   * @param wrapperNamingFunction provides the name of the wrapper for the
   * WebAssembly module
   * @return a new {@code HtmlGenerator} instance
   */
  public HtmlGenerator getHtmlGenerator(Path htmlFile,
      Function<ExpandedComponentInstanceSymbol, String> wasmNamingFunction,
      Function<ExpandedComponentInstanceSymbol, String> wrapperNamingFunction) {
    return new HtmlGenerator(new FileTemplatePrinter(template, htmlFile), wasmNamingFunction,
        wrapperNamingFunction);
  }

}
