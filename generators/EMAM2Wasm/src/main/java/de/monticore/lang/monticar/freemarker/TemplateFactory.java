/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.freemarker;

import freemarker.cache.TemplateLoader;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateExceptionHandler;
import java.io.IOException;

public class TemplateFactory {

  private static final String UTF_8 = "UTF-8";

  private Configuration cfg;
  private TemplateLoader templateLoader;

  public TemplateFactory(TemplateLoader templateLoader) {
    this.templateLoader = templateLoader;
  }

  public Template getTemplate(String name) throws IOException {
    return prepareConfiguration(templateLoader).getTemplate(name);
  }

  private Configuration prepareConfiguration(TemplateLoader templateLoader) throws IOException {
    if (cfg == null) {
      cfg = new Configuration(Configuration.VERSION_2_3_23);
      setUpConfiguration(templateLoader);
    }
    return cfg;
  }

  private void setUpConfiguration(TemplateLoader templateLoader) throws IOException {
    cfg.setTemplateLoader(templateLoader);
    cfg.setDefaultEncoding(UTF_8);
    cfg.setTemplateExceptionHandler(TemplateExceptionHandler.RETHROW_HANDLER);
    cfg.setLogTemplateExceptions(false);
  }
}
