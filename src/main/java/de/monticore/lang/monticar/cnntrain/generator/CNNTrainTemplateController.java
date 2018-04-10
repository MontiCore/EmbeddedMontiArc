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
package de.monticore.lang.monticar.cnntrain.generator;

import de.monticore.lang.monticar.cnntrain._symboltable.*;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;

import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.util.*;

public class CNNTrainTemplateController {

    public static final String FTL_FILE_ENDING = ".ftl";
    public static final String TEMPLATE_CONTROLLER_KEY = "tc";

    private Configuration freemarkerConfig = TemplateConfiguration.get();
    private ConfigurationSymbol configuration;

    private Writer writer;
    private String mainTemplateNameWithoutEnding;
    private Target targetLanguage;

    public CNNTrainTemplateController(ConfigurationSymbol configuration) {
        setConfiguration(configuration);
    }

    public String getFileNameWithoutEnding() {
        return mainTemplateNameWithoutEnding + "_" + getFullConfigurationName();
    }

    public Target getTargetLanguage() {
        return targetLanguage;
    }

    public void setTargetLanguage(Target targetLanguage) {
        this.targetLanguage = targetLanguage;
    }

    public ConfigurationSymbol getConfiguration() {
        return configuration;
    }

    public void setConfiguration(ConfigurationSymbol configuration) {
        this.configuration = configuration;
    }

    public String getConfigurationName() {
        return getConfiguration().getEnclosingScope().getSpanningSymbol().get().getName().replaceAll("\\.", "_");
    }

    public String getFullConfigurationName() {
        return getConfiguration().getEnclosingScope().getSpanningSymbol().get().getFullName().replaceAll("\\.", "_");
    }

    public String getNumEpoch() {
        if (getConfiguration().getNumEpoch() == null) {
            return null;
        }
        return String.valueOf(getConfiguration().getNumEpoch().getValue());
    }

    public String getBatchSize() {
        if (getConfiguration().getNumEpoch() == null) {
            return null;
        }
        return String.valueOf(getConfiguration().getBatchSize().getValue());
    }

    public LoadCheckpointSymbol getLoadCheckpoint() {
        if (getConfiguration().getLoadCheckpoint() == null) {
            return null;
        }
        return getConfiguration().getLoadCheckpoint();
    }

    public String getOptimizerName() {
        if (getConfiguration().getOptimizer() == null) {
            return null;
        }
        return getConfiguration().getOptimizer().getName();
    }

    public Map<String, String> getOptimizerParams() {
        // get classes for single enum values
        List<Class> lrPolicyClasses = new ArrayList<>();
        for (LRPolicy enum_value: LRPolicy.values()) {
            lrPolicyClasses.add(enum_value.getClass());
        }

        Map<String, String>  mapToStrings = new HashMap<>();
        Map<String, OptimizerParamSymbol> optimizerParams = getConfiguration().getOptimizer().getOptimizerParamMap();
        for (Map.Entry<String, OptimizerParamSymbol> entry : optimizerParams.entrySet()) {
            String paramName = entry.getKey();
            String valueAsString = entry.getValue().toString();
            Class realClass = entry.getValue().getValue().getValue().getClass();
            if (realClass == Boolean.class) {
                valueAsString = (Boolean) entry.getValue().getValue().getValue() ? "True" : "False";
            }
            else if (lrPolicyClasses.contains(realClass)) {
                valueAsString = "'" + valueAsString + "'";
            }
            mapToStrings.put(paramName, valueAsString);
        }
        return mapToStrings;
    }


    public void include(String relativePath, String templateWithoutFileEnding, Writer writer){
        String templatePath = relativePath + templateWithoutFileEnding + FTL_FILE_ENDING;

        try {
            Template template = freemarkerConfig.getTemplate(templatePath);
            Map<String, Object> ftlContext = Collections.singletonMap(TEMPLATE_CONTROLLER_KEY, this);

            this.writer = writer;
            template.process(ftlContext, writer);
            this.writer = null;
        }
        catch (IOException e) {
            Log.error("Freemarker could not find template " + templatePath + " :\n" + e.getMessage());
            System.exit(1);
        }
        catch (TemplateException e){
            Log.error("An exception occured in template " + templatePath + " :\n" + e.getMessage());
            System.exit(1);
        }
    }

    public Map.Entry<String,String> process(String templateNameWithoutEnding, Target targetLanguage){
        StringWriter writer = new StringWriter();
        this.mainTemplateNameWithoutEnding = templateNameWithoutEnding;
        this.targetLanguage = targetLanguage;
        include("", templateNameWithoutEnding, writer);

        String fileEnding = targetLanguage.toString();
        if (targetLanguage == Target.CPP){
            fileEnding = ".h";
        }
        String fileName = getFileNameWithoutEnding() + fileEnding;

        Map.Entry<String,String> fileContent = new AbstractMap.SimpleEntry<>(fileName, writer.toString());

        this.mainTemplateNameWithoutEnding = null;
        this.targetLanguage = null;
        return fileContent;
    }
}
