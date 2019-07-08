package de.monticore.lang.monticar.generator.mqtt.template;

import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.Map;

import de.se_rwth.commons.logging.Log;

public class MqttTemplates {

	private static final Template PrettyPrint;
	private static final Template MQTT_CMAKELISTS;
	private static final Template MQTT_ADAPTER_H;
	private static final Template MQTT_ADAPTER_CPP;
	private static final Template MQTT_CALLBACK_H;
	private static final Template MQTT_CALLBACK_CPP;

    // Loading .ftl files
    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(MqttTemplates.class, "");
        try {
        	PrettyPrint = conf.getTemplate("PrettyPrint.ftl");
			MQTT_CMAKELISTS = conf.getTemplate("CMakeLists.ftl");
			MQTT_ADAPTER_H = conf.getTemplate("Adapter.h.ftl");
			MQTT_ADAPTER_CPP = conf.getTemplate("Adapter.cpp.ftl");
			MQTT_CALLBACK_H = conf.getTemplate("Callback.hpp.ftl");
			MQTT_CALLBACK_CPP = conf.getTemplate("Callback.cpp.ftl");
        } catch (IOException e) {
            String msg = "could not load template";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    public static String generateMqttAdapterH(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_ADAPTER_H, data);
    }
    
    public static String generateMqttAdapterCPP(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_ADAPTER_CPP, data);
    }
    
    public static String generateMqttCallbackH(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACK_H, data);
    }
    
    public static String generateMqttCallbackCPP(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACK_CPP, data);
    }
    
    public static String generateMqttCMakeLists(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CMAKELISTS, data);
	}
    
    public static String generatePrettyPrint(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(PrettyPrint, data);
    }

    @SuppressWarnings("rawtypes")
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
