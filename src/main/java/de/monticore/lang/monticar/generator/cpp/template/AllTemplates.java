/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.template;

import de.monticore.lang.monticar.generator.cpp.viewmodel.*;
import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;

public final class AllTemplates {

    private static final Template COMPONENT_STREAM_TEST;
    private static final Template TESTS_MAIN_ENTRY;
    private static final Template STRUCT;
    private static final Template ENUM;
    public static final Template DYNAMIC_INTERFACE_CPP;
    public static final Template DYNAMIC_INTERFACE_H;
    public static final Template TCP_ADAPTER_CPP;
    public static final Template TCP_ADAPTER_H;
    public static final Template DDC_ADAPTER_CPP;
    public static final Template DDC_ADAPTER_ADAPTER_H;
    private static final Template SERVER_WRAPPER;

    private static final Template DYNAMICS_EVENT_PortValueCheker;
    private static final Template DYNAMICS_EVENT_DynamicHelper;

    private static final Template LOG_METHODS;

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template");
        try {
            COMPONENT_STREAM_TEST = conf.getTemplate("/test/ComponentStreamTest2.ftl");
            TESTS_MAIN_ENTRY = conf.getTemplate("/test/TestsMainEntry.ftl");
            STRUCT = conf.getTemplate("/type/Struct.ftl");
            ENUM = conf.getTemplate("/type/Enum.ftl");
            DYNAMIC_INTERFACE_CPP = conf.getTemplate("/dynamic_interface/dynamic_interface.cpp.ftl");
            DYNAMIC_INTERFACE_H = conf.getTemplate("/dynamic_interface/dynamic_interface.h.ftl");
            TCP_ADAPTER_CPP = conf.getTemplate("/dynamic_interface/server_adapter.cpp.ftl");
            TCP_ADAPTER_H = conf.getTemplate("/dynamic_interface/server_adapter.h.ftl");
            DDC_ADAPTER_CPP = conf.getTemplate("/dynamic_interface/ddc_adapter.cpp.ftl");
            DDC_ADAPTER_ADAPTER_H = conf.getTemplate("/dynamic_interface/ddc_adapter.h.ftl");
            SERVER_WRAPPER = conf.getTemplate("/serverwrapper/ServerWrapper.ftl");
            DYNAMICS_EVENT_PortValueCheker = conf.getTemplate("/dynamics/events_port_value_check_h.ftl");
            DYNAMICS_EVENT_DynamicHelper = conf.getTemplate("/dynamics/dynamic_port_request_connect_helper_h.ftl");
            LOG_METHODS = conf.getTemplate("logging/Log.ftl");
        } catch (IOException e) {
            String msg = "could not load templates";
            Log.error(msg, e);
            throw new RuntimeException(msg, e);
        }
    }

    private AllTemplates() {
    }

    public static String generateComponentStreamTest(ComponentStreamTestViewModel viewModel) {
        return generate(COMPONENT_STREAM_TEST, viewModel);
    }

    public static String generateMainEntry(TestsMainEntryViewModel viewModel, boolean backendOctave) {
            return generate(TESTS_MAIN_ENTRY, viewModel);
    }

    public static String generateStruct(StructViewModel viewModel) {
        return generate(STRUCT, viewModel);
    }

    public static String generateEnum(EnumViewModel viewModel) {
        return generate(ENUM, viewModel);
    }

    public static String generateServerWrapper(ServerWrapperViewModel viewModel) {
        return generate(SERVER_WRAPPER, viewModel);
    }

    public static String generateDynamicEventsPortValueCheck(){
        return generateWithoutData(DYNAMICS_EVENT_PortValueCheker);
    }

    public static String generateDynamicHelper(){
        return generateWithoutData(DYNAMICS_EVENT_DynamicHelper);
    }

    public static String generateLogMethods(LoggingViewModel model){
        return generate(LOG_METHODS, model);
    }

    public static String generate(Template template, ViewModelBase viewModelBase) {
        return generate(template, TemplateHelper.getDataForTemplate(viewModelBase));
    }


    public static String generateWithoutData(Template template){
        Log.errorIfNull(template);
        StringWriter sw = new StringWriter();
        try{
            template.process(new Object(), sw);
        } catch (TemplateException | IOException e) {
            Log.error("template generation failed, template: " + template.getName(), e);
        }
        return sw.toString();
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
