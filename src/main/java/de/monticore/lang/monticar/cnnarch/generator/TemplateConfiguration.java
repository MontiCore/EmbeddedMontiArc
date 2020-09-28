/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.util.Map;

public abstract class TemplateConfiguration {
    abstract protected String getBaseTemplatePackagePath();
    abstract public Configuration getConfiguration();

    public TemplateConfiguration() {

    }

    protected Configuration createConfiguration() {
        Configuration configuration = new Configuration(Configuration.VERSION_2_3_23);
        configuration.setClassForTemplateLoading(TemplateConfiguration.class, getBaseTemplatePackagePath());
        configuration.setDefaultEncoding("UTF-8");
        configuration.setTemplateExceptionHandler(TemplateExceptionHandler.RETHROW_HANDLER);

        // numbers in windows germany are formatted as '0,2' instead of '0.2'
        configuration.setNumberFormat("computer");

        return configuration;
    }

    private void quitGeneration(){
        Log.error("Code generation is aborted");
        System.exit(1);
    }

    public void processTemplate(Map<String, Object> ftlContext, String templatePath, Writer writer){
        try{
            Template template = getConfiguration().getTemplate(templatePath);
            template.process(ftlContext, writer);
        } catch (IOException e) {
            Log.error("Freemarker could not find template " + templatePath + " :\n" + e.getMessage());
            quitGeneration();
        } catch (TemplateException e){
            Log.error("An exception occured in template " + templatePath + " :\n" + e.getMessage());
            quitGeneration();
        }
    }

    public String processTemplate(Map<String, Object> ftlContext, String templatePath){
        StringWriter writer = new StringWriter();
        processTemplate(ftlContext, templatePath, writer);
        return writer.toString();
    }
}
