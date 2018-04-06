package de.monticore.lang.monticar.freemarker;

import freemarker.template.TemplateException;
import java.io.IOException;

public interface TemplateProcessor {

  void process(Object dataModel) throws IOException, TemplateException;
}
