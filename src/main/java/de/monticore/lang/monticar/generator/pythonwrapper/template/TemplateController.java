/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import de.monticore.lang.monticar.generator.pythonwrapper.symbolservices.data.ComponentPortInformation;
import de.monticore.lang.monticar.generator.pythonwrapper.template.model.CppWrapperViewModel;
import freemarker.template.TemplateException;

import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.Map;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 *
 */
public class TemplateController {
    private final TemplateLoader templateLoader;
    private final TemplateDataPreparer templateDataPreparer;

    TemplateController(final TemplateLoader templateLoader, final TemplateDataPreparer templateDataPreparer) {
        checkNotNull(templateLoader);
        this.templateLoader = templateLoader;
        this.templateDataPreparer = templateDataPreparer;
    }

    public Map<String, String> processWrapper(final ComponentPortInformation componentPortInformation) {
        Map<String, String> fileContentMap = new HashMap<>();
        StringWriter writer = new StringWriter();
        CppWrapperViewModel viewData = templateDataPreparer.getWrapperViewModel(componentPortInformation);

        String processedFileName = "";
        try {
            processedFileName = componentPortInformation.getComponentName() + "_executor.h";
            templateLoader.processWrapperHeaderTemplate(viewData, writer);
            fileContentMap.put(processedFileName, writer.toString());

            writer = new StringWriter();
            processedFileName = componentPortInformation.getComponentName() + "_executor.cpp";
            templateLoader.processWrapperCppTemplate(viewData, writer);
            fileContentMap.put(processedFileName, writer.toString());

            writer = new StringWriter();
            processedFileName = componentPortInformation.getComponentName() + "_executor.i";
            templateLoader.processSwigTemplate(viewData, writer);
            fileContentMap.put(processedFileName, writer.toString());

            writer = new StringWriter();
            processedFileName = "CMakeLists.txt";
            templateLoader.processCmakeTemplate(viewData, writer);
            fileContentMap.put(processedFileName, writer.toString());

        } catch (TemplateException e) {
            logAndThrowTemplateCreationException("Cannot create template "
                    + processedFileName + ": " + e.getMessage());
        } catch (IOException e) {
            logAndThrowTemplateCreationException("Cannot write template "
                    + processedFileName + " to string: " + e.getMessage());
        }
        return fileContentMap;
    }

    private void logAndThrowTemplateCreationException(String message) {
        Log.error(message);
        throw new TemplateCreationException(message);
    }
}
