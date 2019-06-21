package de.monticore.lang.monticar.generator.mqtt.template;


import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import de.se_rwth.commons.logging.Log;
import java.io.StringWriter;
import java.util.Map;
import java.util.HashMap;

public class MqttTemplates {

	private static final Template Mqtt_CMAKELISTS;
    private static final Template Mqtt_ADAPTER;

    // Choosing ftl files for templates
    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(MqttTemplates.class, "");
        try {
        	Mqtt_CMAKELISTS = conf.getTemplate("CMakeLists.ftl");
        	Mqtt_ADAPTER = conf.getTemplate("Adapter.ftl");
        } catch (IOException e) {
            String msg = "could not load template";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }
    
    /*public static String generateMqttCMakeLists(MqttCMakeListsModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(Mqtt_CMAKELISTS, data);
    }*/
    
    /*public static String generateMqttAdapter(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(Mqtt_ADAPTER, data);
    }*/
    
    private static String generate(Template template, Map dataForTemplate) {
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
