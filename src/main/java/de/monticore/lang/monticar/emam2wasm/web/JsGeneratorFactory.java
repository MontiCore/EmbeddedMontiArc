package de.monticore.lang.monticar.emam2wasm.web;

import de.monticore.lang.monticar.freemarker.FileTemplatePrinter;
import de.monticore.lang.monticar.generator.js.JsGenerator;
import freemarker.template.Template;
import java.nio.file.Path;

/**
 * Factory for {@link JsGenerator}.
 */
public class JsGeneratorFactory {

  private Template template;

  /**
   * Creates a new factory
   *
   * @param template the template that the {@link JsGenerator} will use to
   * produce HTML code
   */
  public JsGeneratorFactory(Template template) {
    this.template = template;
  }

  /**
   * Produces a new {@link JsGenerator}.
   *
   * @param jsFile path to the file that the generator will write to
   * @return a new {@code JsGenerator} instance
   */
  public JsGenerator getJsGenerator(Path jsFile) {
    return new JsGenerator(new FileTemplatePrinter(template, jsFile));
  }

}
