package de.monticore.lang.monticar.generator.cpp.template;

import de.monticore.lang.monticar.generator.cpp.viewmodel.ComponentStreamTestViewModel;
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

    static {
        Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
        conf.setDefaultEncoding("UTF-8");
        conf.setTemplateExceptionHandler(TemplateExceptionHandler.DEBUG_HANDLER);
        conf.setLogTemplateExceptions(false);
        conf.setClassForTemplateLoading(AllTemplates.class, "/template");
        try {
            COMPONENT_STREAM_TEST = conf.getTemplate("/test/ComponentStreamTest.ftl");
            TESTS_MAIN_ENTRY = conf.getTemplate("/test/TestsMainEntry.ftl");
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

    public static String generateMainEntry(TestsMainEntryViewModel viewModel) {
        return generate(TESTS_MAIN_ENTRY, viewModel);
    }

    private static String generate(Template template, ViewModelBase viewModelBase) {
        return generate(template, TemplateHelper.getDataForTemplate(viewModelBase));
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
