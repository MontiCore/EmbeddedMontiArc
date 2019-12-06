/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.util.Map;

public class TemplateConfiguration {

    private static TemplateConfiguration instance;
    private Configuration configuration;

    private TemplateConfiguration() {
        configuration = new Configuration(Configuration.VERSION_2_3_23);
        configuration.setClassForTemplateLoading(TemplateConfiguration.class, "/templates/caffe2/");
        configuration.setDefaultEncoding("UTF-8");
        configuration.setTemplateExceptionHandler(TemplateExceptionHandler.RETHROW_HANDLER);
    }

    private static void quitGeneration(){
        Log.error("Code generation is aborted");
        System.exit(1);
    }

    public Configuration getConfiguration() {
        return configuration;
    }

    public static Configuration get(){
        if (instance == null){
            instance = new TemplateConfiguration();
        }
        return instance.getConfiguration();
    }

    public static void processTemplate(Map<String, Object> ftlContext, String templatePath, Writer writer){
        try{
            Template template = TemplateConfiguration.get().getTemplate(templatePath);
            template.process(ftlContext, writer);
        } catch (IOException e) {
            Log.error("Freemarker could not find template " + templatePath + " :\n" + e.getMessage());
            quitGeneration();
        } catch (TemplateException e){
            Log.error("An exception occured in template " + templatePath + " :\n" + e.getMessage());
            quitGeneration();
        }
    }

    public static String processTemplate(Map<String, Object> ftlContext, String templatePath){
        StringWriter writer = new StringWriter();
        processTemplate(ftlContext, templatePath, writer);
        return writer.toString();
    }

}
