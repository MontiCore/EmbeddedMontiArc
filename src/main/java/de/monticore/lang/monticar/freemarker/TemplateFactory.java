package de.monticore.lang.monticar.freemarker;

import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateExceptionHandler;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

public class TemplateFactory {

  private static final String UTF_8 = "UTF-8";

  private Configuration cfg;
  private Path templateDir;

  public TemplateFactory(Path templateDir) {
    this.templateDir = templateDir;
  }

  public Template getTemplate(String name) throws IOException {
    return prepareConfiguration(templateDir).getTemplate(name);
  }

  private Configuration prepareConfiguration(Path dir) throws IOException {
    if (cfg == null) {
      cfg = new Configuration(Configuration.VERSION_2_3_23);
      setUpConfiguration(dir.toFile());
    }
    return cfg;
  }

  private void setUpConfiguration(File templateDir) throws IOException {
    cfg.setDirectoryForTemplateLoading(templateDir);
    cfg.setDefaultEncoding(UTF_8);
    cfg.setTemplateExceptionHandler(TemplateExceptionHandler.RETHROW_HANDLER);
    cfg.setLogTemplateExceptions(false);
  }
}
