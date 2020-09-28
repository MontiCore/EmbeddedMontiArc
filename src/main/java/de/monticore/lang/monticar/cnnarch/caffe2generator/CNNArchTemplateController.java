/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnnarch.generator.LayerNameCreator;
import de.monticore.lang.monticar.cnnarch.generator.Target;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.predefined.Sigmoid;
import de.monticore.lang.monticar.cnnarch.predefined.Softmax;
import de.monticore.lang.monticar.generator.FileContent;

import java.io.StringWriter;
import java.io.Writer;
import java.util.*;

public class CNNArchTemplateController {

    public static final String FTL_FILE_ENDING = ".ftl";
    public static final String TEMPLATE_ELEMENTS_DIR_PATH = "elements/";
    public static final String TEMPLATE_CONTROLLER_KEY = "tc";
    public static final String ELEMENT_DATA_KEY = "element";

    private LayerNameCreator nameManager;
    private ArchitectureSymbol architecture;
    private String loss;

    //temporary attributes. They are set after calling process()
    private Writer writer;
    private String mainTemplateNameWithoutEnding;
    private Target targetLanguage;
    private ArchitectureElementData dataElement;

    public static final String CROSS_ENTROPY = "cross_entropy";
    public static final String EUCLIDEAN = "euclidean";

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

    public String getDataPath(){
        return getArchitecture().getDataPath();
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
        for (VariableSymbol element : getArchitecture().getInputs()){
            list.add(nameManager.getName(element));
        }
        return list;
    }

    public List<String> getArchitectureOutputs(){
        List<String> list = new ArrayList<>();
        for (VariableSymbol element : getArchitecture().getOutputs()){
            list.add(nameManager.getName(element));
        }
        return list;
    }

    public String getComponentName(){
        return getArchitecture().getComponentName();
    }

    public String getArchitectureLoss(){
        return this.loss;
    }

    public void include(String relativePath, String templateWithoutFileEnding, Writer writer){
        String templatePath = relativePath + templateWithoutFileEnding + FTL_FILE_ENDING;
        Map<String, Object> ftlContext = new HashMap<>();
        ftlContext.put(TEMPLATE_CONTROLLER_KEY, this);
        ftlContext.put(ELEMENT_DATA_KEY, getCurrentElement());
        TemplateConfiguration.processTemplate(ftlContext, templatePath, writer);
    }

    public void include(VariableSymbol element, Writer writer){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(element);

        if (element.isAtomic()){
            if (element.isInput()){
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Input", writer);
            } else {
                include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer);
            }
        } else {
            include((ArchitectureElementSymbol) element.getResolvedThis().get(), writer);
        }

        setCurrentElement(previousElement);
    }

    public void include(LayerSymbol layer, Writer writer){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(layer);

        if (layer.isAtomic()){
            ArchitectureElementSymbol nextElement = layer.getOutputElement().get();
            if (!isSoftmaxOutput(nextElement) && !isLogisticRegressionOutput(nextElement)){
                String templateName = layer.getDeclaration().getName();
                include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer);
            }
        } else {
            include((ArchitectureElementSymbol) layer.getResolvedThis().get(), writer);
        }

        setCurrentElement(previousElement);
    }

    public void include(CompositeElementSymbol compositeElement, Writer writer){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(compositeElement);

        for (ArchitectureElementSymbol element : compositeElement.getElements()){
            include(element, writer);
        }

        setCurrentElement(previousElement);
    }

    public void include(ArchitectureElementSymbol architectureElement, Writer writer){
        if (architectureElement instanceof CompositeElementSymbol){
            include((CompositeElementSymbol) architectureElement, writer);
        } else if (architectureElement instanceof LayerSymbol){
            include((LayerSymbol) architectureElement, writer);
        } else {
            include((VariableSymbol) architectureElement, writer);
        }
    }

    public void include(ArchitectureElementSymbol architectureElement){
        if (writer == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, writer);
    }

    public FileContent process(String templateNameWithoutEnding, Target targetLanguage) {
        StringWriter newWriter = new StringWriter();
        this.mainTemplateNameWithoutEnding = templateNameWithoutEnding;
        this.targetLanguage = targetLanguage;
        this.writer = newWriter;
        this.include("", templateNameWithoutEnding, newWriter);
        String fileEnding = targetLanguage.toString();
        String fileName = this.getFileNameWithoutEnding() + fileEnding;
        FileContent fileContent = new FileContent(newWriter.toString(), fileName);
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
        if (isTOutput(Sigmoid.class, architectureElement)){
            this.loss = CROSS_ENTROPY;
            return true;
        }
        return false;
    }

    public boolean isLinearRegressionOutput(ArchitectureElementSymbol architectureElement){
        if (architectureElement.isOutput()
                && !isLogisticRegressionOutput(architectureElement)
                && !isSoftmaxOutput(architectureElement)){
            this.loss = EUCLIDEAN;
            return true;
        }
        return false;
    }


    public boolean isSoftmaxOutput(ArchitectureElementSymbol architectureElement){
        if (isTOutput(Softmax.class, architectureElement)){
            this.loss = CROSS_ENTROPY;
            return true;
        }
        return false;
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
