/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.someip.template;

import de.se_rwth.commons.logging.Log;

import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.Map;

public class SomeIPTemplates {

	private static final Template SOMEIP_PRETTYPRINT;
	private static final Template SOMEIP_CMAKELISTS;
	private static final Template SOMEIP_ADAPTER_H;
	private static final Template SOMEIP_ADAPTER_CPP;

    // Loading .ftl files
    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(SomeIPTemplates.class, "");
        try {
        	SOMEIP_PRETTYPRINT = conf.getTemplate("PrettyPrint.ftl");
        	SOMEIP_CMAKELISTS = conf.getTemplate("CMakeLists.ftl");
        	SOMEIP_ADAPTER_H = conf.getTemplate("Adapter.h.ftl");
			SOMEIP_ADAPTER_CPP = conf.getTemplate("Adapter.cpp.ftl");
        } catch (IOException e) {
            String msg = "could not load template";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    public static String generateSomeIPAdapterH(SomeIPAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(SOMEIP_ADAPTER_H, data);
    }

    public static String generateSomeIPAdapterCPP(SomeIPAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(SOMEIP_ADAPTER_CPP, data);
    }

    public static String generatePrettyPrint(SomeIPAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(SOMEIP_PRETTYPRINT, data);
    }

	public static String generateSomeIPCMakeLists(SomeIPAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(SOMEIP_CMAKELISTS, data);
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
