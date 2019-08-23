/* (c) https://github.com/MontiCore/monticore */
package de.monitcore.lang.monticar.utilities.tools;

import de.se_rwth.commons.logging.Log;
import freemarker.template.Template;
import freemarker.template.TemplateException;

import java.io.IOException;
import java.io.StringWriter;

public final class AllTemplates {
    private AllTemplates() {
    }
    public static String generate(Template template, Object dataForTemplate) {
        Log.errorIfNull(template);
        Log.errorIfNull(dataForTemplate);
        StringWriter sw = new StringWriter();
        try {
            template.process(dataForTemplate, sw);
        } catch (TemplateException | IOException e) {
            Log.error("template generation failed, template: " + template.getName(), e);
        }
        return sw.toString();
    }

}
