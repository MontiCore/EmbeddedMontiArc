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
package de.monticore.lang.monticar.emadl.generator;

/*import de.monticore.lang.monticar.cnnarch._ast.ASTArgumentRhs;
import de.monticore.lang.monticar.cnnarch._ast.ASTLayer;
import de.monticore.lang.monticar.cnnarch._ast.ASTMethod;
import de.monticore.lang.monticar.cnntrain._ast.ASTParameterRhs;
import de.monticore.lang.monticar.cnntrain._ast.ASTTrainingConfiguration;
import de.monticore.lang.monticar.emadl.helper.ASTPrinter;*/

import freemarker.template.Configuration;

import java.util.HashMap;
import java.util.Map;

public class TemplateController {

    public static final String TEMPLATE_CONTROLLER_KEY = "tc";
    public static final String TEMPLATE_METHOD_KEY = "method";
    public static final String TEMPLATE_LAYERS_KEY = "layers";
    public static final String TEMPLATE_MAIN_LAYERS_KEY = "mainLayers";
    public static final String TEMPLATE_CONF_KEY = "conf";
    public static final String TEMPLATE_TARGET_KEY = "target";
    public static final String TEMPLATE_CONTEXT_KEY = "context";
    public static final String TEMPLATE_INPUT_PORT_KEY = "inputPort";
    public static final String TEMPLATE_OUTPUT_PORT_KEY = "outputPort";
    public static final String TEMPLATE_OUTPUT_LAYER_KEY = "outputLayer";
    public static final String TEMPLATE_LAYER_KEY = "layer";
    public static final String TEMPLATE_PREVIOUS_LAYER_KEY = "previous";

    public static final String TEMPLATE_METHOD_DIR_PATH = "methods/";
    public static final String FTL_FILE_ENDING = ".ftl";

    private Configuration freemarkerConfig;
    private TemplateData data;
    private Map<String,Object> ftlContext;


    public TemplateController(TemplateData data, Configuration freemarkerConfig) {
        this.data = data;
        this.freemarkerConfig = freemarkerConfig;
        this.ftlContext = new HashMap<>();
        ftlContext.put(TEMPLATE_CONTROLLER_KEY, this);
        /*ftlContext.put(TEMPLATE_LAYERS_KEY, data.getLayers());
        ftlContext.put(TEMPLATE_MAIN_LAYERS_KEY, data.getMainLayers());*/
        ftlContext.put(TEMPLATE_CONF_KEY, data.getConfig());
        /*ftlContext.put(TEMPLATE_TARGET_KEY, data.getTarget().name());
        ftlContext.put(TEMPLATE_CONTEXT_KEY, data.getContext());
        ftlContext.put(TEMPLATE_OUTPUT_LAYER_KEY, data.getOutputLayer());*/
        ftlContext.put(TEMPLATE_INPUT_PORT_KEY, data.getInputPort());
        ftlContext.put(TEMPLATE_OUTPUT_PORT_KEY, data.getOutputPort());
    }


    /*public void process(FileWriter writer) throws IOException, TemplateException {

        String templateName = "";
        switch (data.getTarget()) {
            case CPLUSPLUS:
                templateName = "MxNetCpp.ftl";
                break;
            case PYTHON:
                templateName = "MxNetPython.ftl";
                break;
        }
        Template template = freemarkerConfig.getTemplate(templateName);
        template.process(ftlContext, writer);
    }


    public String print(ASTArgumentRhs rhs) {
        return ASTPrinter.toString(rhs);
    }

    public String print(ASTParameterRhs rhs) {
        return ASTPrinter.toString(rhs);
    }


    public String print(ASTMethod method, String key) {
        ASTArgumentRhs rhs = method.get(key);
        return print(rhs);
    }

    public String print(ASTTrainingConfiguration conf, String key) {
        ASTParameterRhs rhs = conf.get(key);
        return print(rhs);
    }

    public String include(ASTLayer layer) {

        String templateName = layer.getMethod().getName() + FTL_FILE_ENDING;
        StringWriter stringWriter = new StringWriter();
        Object previous;
        int pos = data.getLayers().indexOf(layer);

        if (pos == 0) {
            previous = null;
        }
        else {
            previous = data.getLayers().get(pos - 1);
        }

        ftlContext.put(TEMPLATE_LAYER_KEY, layer);
        ftlContext.put(TEMPLATE_PREVIOUS_LAYER_KEY, previous);
        ftlContext.put(TEMPLATE_METHOD_KEY, layer.getMethod());

        try {
            Template template = freemarkerConfig.getTemplate(TEMPLATE_METHOD_DIR_PATH + templateName);
            template.process(ftlContext, stringWriter);
            return stringWriter.toString();
        }
        catch (IOException e) {
            Log.error("Freemarker could not find template " + templateName + " :\n" + e.getMessage());
            return "<IOException>";
        }
        catch (TemplateException e){
            Log.error("An exception occured in template " + templateName + " :\n" + e.getMessage());
            return "<TemplateException>";
        }
    }

    public String name(ASTLayer layer) {
        String res;
        if (layer == null) {
            res = "input";
        }
        else {
            res = layer.getMethod().getName() + data.getLayers().indexOf(layer);
        }
        return res;
    }*/

}
