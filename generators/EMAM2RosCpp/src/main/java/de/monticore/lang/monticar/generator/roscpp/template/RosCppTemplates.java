/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.template;

import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.Map;

public class RosCppTemplates {

    private RosCppTemplates() {
    }

    private static final Template ROSCPP_CMAKELISTS;
    private static final Template ROS_ADAPTER;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(RosCppTemplates.class, "");
        try {
            ROSCPP_CMAKELISTS = conf.getTemplate("CMakeLists.ftl");
            ROS_ADAPTER = conf.getTemplate("Adapter.ftl");
        } catch (IOException e) {
            String msg = "could not load template";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    public static String generateRosCMakeLists(RosCppCMakeListsModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(ROSCPP_CMAKELISTS, data);
    }

    public static String generateRosAdapter(RosAdapterModel model) {
        HashMap<String, Object> data = new HashMap<>();
        data.put("model", model);
        return generate(ROS_ADAPTER, data);
    }

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
