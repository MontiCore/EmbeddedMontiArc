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
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.Sigmoid;
import de.monticore.lang.monticar.cnnarch.predefined.Softmax;

import java.io.StringWriter;
import java.io.Writer;
import java.util.*;

public class CNNArchTemplateController {

    public static final String FTL_FILE_ENDING = ".ftl";
    public static final String TEMPLATE_ELEMENTS_DIR_PATH = "elements/";
    public static final String TEMPLATE_CONTROLLER_KEY = "tc";
    public static final String ELEMENT_DATA_KEY = "element";
    public static final String NET_DEFINITION_MODE_KEY = "definition_mode";

    private LayerNameCreator nameManager;
    private ArchitectureSymbol architecture;

    //temporary attributes. They are set after calling process()
    private Writer writer;
    private String mainTemplateNameWithoutEnding;
    private Target targetLanguage;
    private ArchitectureElementData dataElement;


    public CNNArchTemplateController(ArchitectureSymbol architecture) {
        setArchitecture(architecture);
    }

    public String getFileNameWithoutEnding() {
        return mainTemplateNameWithoutEnding + "_" + getFullArchitectureName();
    }

    public ArchitectureElementData getCurrentElement() {
        return dataElement;
    }

    public void setCurrentElement(ArchitectureElementSymbol layer) {
        this.dataElement = new ArchitectureElementData(getName(layer), layer, this);
    }

    public void setCurrentElement(ArchitectureElementData dataElement) {
        this.dataElement = dataElement;
    }

    public ArchitectureSymbol getArchitecture() {
        return architecture;
    }

    public void setArchitecture(ArchitectureSymbol architecture) {
        this.architecture = architecture;
        this.nameManager = new LayerNameCreator(architecture);
    }

    public String getName(ArchitectureElementSymbol layer){
        return nameManager.getName(layer);
    }

    public String getArchitectureName(){
        return getArchitecture().getEnclosingScope().getSpanningSymbol().get().getName().replaceAll("\\.","_");
    }

    public String getFullArchitectureName(){
        return getArchitecture().getEnclosingScope().getSpanningSymbol().get().getFullName().replaceAll("\\.","_");
    }

    public List<String> getLayerInputs(ArchitectureElementSymbol layer){
        List<String> inputNames = new ArrayList<>();

        if (isSoftmaxOutput(layer) || isLogisticRegressionOutput(layer)){
            inputNames = getLayerInputs(layer.getInputElement().get());
        } else {
            for (ArchitectureElementSymbol input : layer.getPrevious()) {
                if (input.getOutputTypes().size() == 1) {
                    inputNames.add(getName(input));
                } else {
                    for (int i = 0; i < input.getOutputTypes().size(); i++) {
                        inputNames.add(getName(input) + "[" + i + "]");
                    }
                }
            }
        }
        return inputNames;

    }

    public List<String> getArchitectureInputs(){
        List<String> list = new ArrayList<>();
        for (IOSymbol ioElement : getArchitecture().getInputs()){
            list.add(nameManager.getName(ioElement));
        }
        return list;
    }

    public List<String> getArchitectureOutputs(){
        List<String> list = new ArrayList<>();
        for (IOSymbol ioElement : getArchitecture().getOutputs()){
            list.add(nameManager.getName(ioElement));
        }
        return list;
    }

    public void include(String relativePath, String templateWithoutFileEnding, Writer writer, NetDefinitionMode netDefinitionMode){
        String templatePath = relativePath + templateWithoutFileEnding + FTL_FILE_ENDING;
        Map<String, Object> ftlContext = new HashMap<>();
        ftlContext.put(TEMPLATE_CONTROLLER_KEY, this);
        ftlContext.put(ELEMENT_DATA_KEY, getCurrentElement());
        ftlContext.put(NET_DEFINITION_MODE_KEY, netDefinitionMode);
        TemplateConfiguration.processTemplate(ftlContext, templatePath, writer);
    }

    public void include(String relativePath, String templateWithoutFileEnding, Writer writer) {
        String templatePath = relativePath + templateWithoutFileEnding + FTL_FILE_ENDING;
        Map<String, Object> ftlContext = new HashMap<>();
        ftlContext.put(TEMPLATE_CONTROLLER_KEY, this);
        ftlContext.put(ELEMENT_DATA_KEY, getCurrentElement());
        TemplateConfiguration.processTemplate(ftlContext, templatePath, writer);
    }

    public void include(IOSymbol ioElement, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(ioElement);

        if (ioElement.isAtomic()){
            if (ioElement.isInput()){
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Input", writer, netDefinitionMode);
            } else {
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer, netDefinitionMode);
            }
        } else {
            include(ioElement.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(LayerSymbol layer, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(layer);

        if (layer.isAtomic()){
            ArchitectureElementSymbol nextElement = layer.getOutputElement().get();
            if (!isSoftmaxOutput(nextElement) && !isLogisticRegressionOutput(nextElement)){
                String templateName = layer.getDeclaration().getName();
                include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
            }
        } else {
            include(layer.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(CompositeElementSymbol compositeElement, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(compositeElement);

        for (ArchitectureElementSymbol element : compositeElement.getElements()){
            include(element, writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(ArchitectureElementSymbol architectureElement, Writer writer, NetDefinitionMode netDefinitionMode){
        if (architectureElement instanceof CompositeElementSymbol){
            include((CompositeElementSymbol) architectureElement, writer, netDefinitionMode);
        } else if (architectureElement instanceof LayerSymbol){
            include((LayerSymbol) architectureElement, writer, netDefinitionMode);
        } else {
            include((IOSymbol) architectureElement, writer, netDefinitionMode);
        }
    }

    public void include(ArchitectureElementSymbol architectureElementSymbol, String netDefinitionMode) {
        include(architectureElementSymbol, NetDefinitionMode.fromString(netDefinitionMode));
    }

    public void include(ArchitectureElementSymbol architectureElement, NetDefinitionMode netDefinitionMode){
        if (writer == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, writer, netDefinitionMode);
    }

    public Map.Entry<String,String> process(String templateNameWithoutEnding, Target targetLanguage){
        StringWriter newWriter = new StringWriter();
        this.mainTemplateNameWithoutEnding = templateNameWithoutEnding;
        this.targetLanguage = targetLanguage;
        this.writer = newWriter;

        include("", templateNameWithoutEnding, newWriter);
        String fileEnding = targetLanguage.toString();
        String fileName = getFileNameWithoutEnding() + fileEnding;
        Map.Entry<String,String> fileContent = new AbstractMap.SimpleEntry<>(fileName, newWriter.toString());

        this.mainTemplateNameWithoutEnding = null;
        this.targetLanguage = null;
        this.writer = null;
        return fileContent;
    }

    public String join(Iterable iterable, String separator){
        return join(iterable, separator, "", "");
    }

    public String join(Iterable iterable, String separator, String elementPrefix, String elementPostfix){
        StringBuilder stringBuilder = new StringBuilder();
        boolean isFirst = true;
        for (Object element : iterable){
            if (!isFirst){
                stringBuilder.append(separator);
            }
            stringBuilder.append(elementPrefix);
            stringBuilder.append(element.toString());
            stringBuilder.append(elementPostfix);
            isFirst = false;
        }
        return stringBuilder.toString();
    }


    public boolean isLogisticRegressionOutput(ArchitectureElementSymbol architectureElement){
        return isTOutput(Sigmoid.class, architectureElement);
    }

    public boolean isLinearRegressionOutput(ArchitectureElementSymbol architectureElement){
        return architectureElement.isOutput()
                && !isLogisticRegressionOutput(architectureElement)
                && !isSoftmaxOutput(architectureElement);
    }


    public boolean isSoftmaxOutput(ArchitectureElementSymbol architectureElement){
        return isTOutput(Softmax.class, architectureElement);
    }

    private boolean isTOutput(Class inputPredefinedLayerClass, ArchitectureElementSymbol architectureElement){
        if (architectureElement.isOutput()
                && architectureElement.getInputElement().isPresent()
                && architectureElement.getInputElement().get() instanceof LayerSymbol){
            LayerSymbol inputLayer = (LayerSymbol) architectureElement.getInputElement().get();
            if (inputPredefinedLayerClass.isInstance(inputLayer.getDeclaration())){
                return true;
            }
        }
        return false;
    }
}
