package de.monticore.lang.monticar.generator.mqtt.template;

import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import de.se_rwth.commons.logging.Log;

@SuppressWarnings("unused")
public class MqttTemplates {

	private static final Template PrettyPrint;

    // Choosing .ftl files for templates
    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(MqttTemplates.class, "");
        try {
        	PrettyPrint = conf.getTemplate("PrettyPrint.ftl");
        } catch (IOException e) {
            String msg = "could not load template";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }
}
