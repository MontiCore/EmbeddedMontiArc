/* (c) https://github.com/MontiCore/monticore */
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

	private static final Template MQTT_PRETTYPRINT;
	private static final Template MQTT_CMAKELISTS;
	private static final Template MQTT_FINDMQTT;
	private static final Template MQTT_ADAPTER_H;
	private static final Template MQTT_ADAPTER_CPP;
	private static final Template MQTT_CALLBACKQ_H;
	private static final Template MQTT_CALLBACKQ_CPP;
	private static final Template MQTT_CALLBACKZ_H;
	private static final Template MQTT_CALLBACKZ_CPP;
	private static final Template MQTT_CALLBACKN_H;
	private static final Template MQTT_CALLBACKN_CPP;
	private static final Template MQTT_CALLBACKB_H;
	private static final Template MQTT_CALLBACKB_CPP;

    // Loading .ftl files
    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(MqttTemplates.class, "");
        try {
        	MQTT_PRETTYPRINT = conf.getTemplate("PrettyPrint.ftl");
			MQTT_CMAKELISTS = conf.getTemplate("CMakeLists.ftl");
			MQTT_FINDMQTT = conf.getTemplate("FindMqtt.ftl");
			MQTT_ADAPTER_H = conf.getTemplate("Adapter.h.ftl");
			MQTT_ADAPTER_CPP = conf.getTemplate("Adapter.cpp.ftl");
			MQTT_CALLBACKQ_H = conf.getTemplate("CallbackQ.hpp.ftl");
			MQTT_CALLBACKQ_CPP = conf.getTemplate("CallbackQ.cpp.ftl");
			MQTT_CALLBACKZ_H = conf.getTemplate("CallbackZ.hpp.ftl");
			MQTT_CALLBACKZ_CPP = conf.getTemplate("CallbackZ.cpp.ftl");
			MQTT_CALLBACKN_H = conf.getTemplate("CallbackN.hpp.ftl");
			MQTT_CALLBACKN_CPP = conf.getTemplate("CallbackN.cpp.ftl");
			MQTT_CALLBACKB_H = conf.getTemplate("CallbackB.hpp.ftl");
			MQTT_CALLBACKB_CPP = conf.getTemplate("CallbackB.cpp.ftl");
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

    public static String generateMqttCallbackQH(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKQ_H, data);
    }

    public static String generateMqttCallbackQCPP(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKQ_CPP, data);
    }

		public static String generateMqttCallbackNH(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKN_H, data);
    }

    public static String generateMqttCallbackNCPP(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKN_CPP, data);
    }

		public static String generateMqttCallbackZH(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKZ_H, data);
    }

    public static String generateMqttCallbackZCPP(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKZ_CPP, data);
    }

		public static String generateMqttCallbackBH(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKB_H, data);
    }

    public static String generateMqttCallbackBCPP(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CALLBACKB_CPP, data);
    }

    public static String generateMqttCMakeLists(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_CMAKELISTS, data);
	}

    public static String generateMqttFindMqtt(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_FINDMQTT, data);
	}

    public static String generatePrettyPrint(MqttAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(MQTT_PRETTYPRINT, data);
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
