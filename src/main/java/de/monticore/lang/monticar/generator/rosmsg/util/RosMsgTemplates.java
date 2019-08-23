/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg.util;

import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.util.HashMap;

public class RosMsgTemplates {

    private RosMsgTemplates(){
    }

    private static final Template ROS2_CMAKELISTS;
    private static final Template ROS2_PACKAGE_XML;
    private static final Template ROS_CMAKELISTS;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(RosMsgTemplates.class, "/templates");
        try {
            ROS2_CMAKELISTS = conf.getTemplate("ros2.MsgGen.CMakeLists.ftl");
            ROS_CMAKELISTS = conf.getTemplate("ros.MsgGen.CMakeLists.ftl");
            ROS2_PACKAGE_XML = conf.getTemplate("ros2.MsgGen.Package.ftl");
        } catch (IOException e) {
            String msg = "could not load templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    public static String generateRos2CMakeLists(CMakeListsViewModel viewModel){
        HashMap<String, Object> data = new HashMap<>();
        data.put("viewModel", viewModel);
        return generate(ROS2_CMAKELISTS, data);
    }

    public static String generateRosCMakeLists(CMakeListsViewModel viewModel){
        HashMap<String, Object> data = new HashMap<>();
        data.put("viewModel", viewModel);
        return generate(ROS_CMAKELISTS, data);
    }

    public static String generateRos2Package(){
        return generate(ROS2_PACKAGE_XML, new HashMap<>());
    }

    private static String generate(Template template, Object dataForTemplate) {
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
