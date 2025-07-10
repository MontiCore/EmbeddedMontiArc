/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppWrapperViewModel;
import de.se_rwth.commons.logging.Log;
import freemarker.core.ParseException;
import freemarker.template.*;

import java.io.IOException;
import java.io.Writer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 *
 */
class TemplateLoader {
    // Templates
    private final static String SWIG_INTERFACE = "SWIGInterface.ftlh";
    private final static String WRAPPER_CPP = "WrapperCpp.ftlh";
    private final static String WRAPPER_HEADER = "WrapperCppHeader.ftlh";
    private final static String CMAKE = "Cmake.ftlh";

    private static final List<String> TEMPLATE_FILES = Lists.newArrayList(
            SWIG_INTERFACE,
            WRAPPER_CPP,
            WRAPPER_HEADER,
            CMAKE
    );

    private final Configuration configuration;


    private boolean templatesLoaded;
    private final Map<String, Template> templates;

    public TemplateLoader(Configuration configuration) {
        this.templatesLoaded = false;
        this.configuration = configuration;
        this.templates = new HashMap<>();
    }

    private void loadTemplates() {
        if (!templatesLoaded) {
            for (String templateFile : TEMPLATE_FILES) {
                try {
                    Template template = configuration.getTemplate(templateFile);
                    templates.put(templateFile, template);
                } catch (MalformedTemplateNameException e) {
                    logAndThrow("Template file " + templateFile + " name is malformed");
                } catch (ParseException e) {
                    logAndThrow("Could not parse template file " + templateFile);
                } catch (TemplateNotFoundException e) {
                    logAndThrow("Template file " + templateFile + " cannot be found");
                } catch (IOException e) {
                    logAndThrow("Cannot read template file " + templateFile);
                }
            }
        }
        templatesLoaded = true;
    }

    public void processSwigTemplate(CppWrapperViewModel data, Writer writer) throws IOException, TemplateException {
        loadTemplates();
        getSwigInterfaceTemplate().process(data, writer);
    }

    public void processWrapperCppTemplate(CppWrapperViewModel data, Writer writer) throws IOException, TemplateException {
        loadTemplates();
        getWrapperCppTemplate().process(data, writer);
    }

    public void processWrapperHeaderTemplate(CppWrapperViewModel data, Writer writer) throws IOException, TemplateException {
        loadTemplates();
        getWrapperCppHeaderTemplate().process(data, writer);
    }

    public void processCmakeTemplate(CppWrapperViewModel data, Writer writer) throws IOException, TemplateException {
        loadTemplates();
        getCmakeTemplate().process(data, writer);
    }

    private Template getSwigInterfaceTemplate() {
        loadTemplates();
        return this.templates.get(SWIG_INTERFACE);
    }

    private Template getWrapperCppTemplate() {
        loadTemplates();
        return this.templates.get(WRAPPER_CPP);
    }

    private Template getWrapperCppHeaderTemplate() {
        loadTemplates();
        return this.templates.get(WRAPPER_HEADER);
    }

    private Template getCmakeTemplate() {
        loadTemplates();
        return this.templates.get(CMAKE);
    }

    private void logAndThrow(String message) {
        Log.error(message);
        throw new TemplateLoadingException(message);
    }
}
