/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator.cpp.template;

import de.monticore.lang.monticar.generator.cpp.viewmodel.AutopilotAdapterDataModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ComponentStreamTestViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.EnumViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ServerWrapperViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.StructViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.TestsMainEntryViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ViewModelBase;
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
    private static final Template AUTOPILOT_ADAPTER_CPP;
    private static final Template AUTOPILOT_ADAPTER_H;
    private static final Template SERVER_WRAPPER;

    private static final Template DYNAMICS_EVENT_PortValueCheker;
    private static final Template DYNAMICS_EVENT_DynamicHelper;


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
            AUTOPILOT_ADAPTER_CPP = conf.getTemplate("/autopilotadapter/AutopilotAdapterCpp.ftl");
            AUTOPILOT_ADAPTER_H = conf.getTemplate("/autopilotadapter/AutopilotAdapterH.ftl");
            SERVER_WRAPPER = conf.getTemplate("/serverwrapper/ServerWrapper.ftl");
            DYNAMICS_EVENT_PortValueCheker = conf.getTemplate("/dynamics/events_port_value_check_h.ftl");
            DYNAMICS_EVENT_DynamicHelper = conf.getTemplate("/dynamics/dynamic_port_request_connect_helper_h.ftl");
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

    public static String generateAutopilotAdapterCpp(AutopilotAdapterDataModel viewModel) {
        return generate(AUTOPILOT_ADAPTER_CPP, viewModel);
    }
    public static String generateAutopilotAdapterH(AutopilotAdapterDataModel viewModel) {
        return generate(AUTOPILOT_ADAPTER_H, viewModel);
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


    private static String generate(Template template, ViewModelBase viewModelBase) {
        return generate(template, TemplateHelper.getDataForTemplate(viewModelBase));
    }


    private static String generateWithoutData(Template template){
        Log.errorIfNull(template);
        StringWriter sw = new StringWriter();
        try{
            template.process(new Object(), sw);
        } catch (TemplateException | IOException e) {
            Log.error("template generation failed, template: " + template.getName(), e);
        }
        return sw.toString();
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
