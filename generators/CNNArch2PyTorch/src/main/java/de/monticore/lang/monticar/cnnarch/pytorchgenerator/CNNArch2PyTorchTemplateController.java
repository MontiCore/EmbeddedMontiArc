package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.generator.ArchitectureElementData;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchTemplateController;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;

import java.io.Writer;
import java.util.*;

public class CNNArch2PyTorchTemplateController extends CNNArchTemplateController {
    public static final String NET_DEFINITION_MODE_KEY = "mode";
    public List<String> worked_list = new ArrayList<String>();

    public CNNArch2PyTorchTemplateController(ArchitectureSymbol architecture,
                                             TemplateConfiguration templateConfiguration) {
        super(architecture, templateConfiguration);
    }

    public void include(String relativePath, String templateWithoutFileEnding, Writer writer, NetDefinitionMode netDefinitionMode){
        String templatePath = relativePath + templateWithoutFileEnding + FTL_FILE_ENDING;
        Map<String, Object> ftlContext = new HashMap<>();
        ftlContext.put(TEMPLATE_CONTROLLER_KEY, this);
        ftlContext.put(ELEMENT_DATA_KEY, getCurrentElement());
        ftlContext.put(NET_DEFINITION_MODE_KEY, netDefinitionMode.toString());

        if (this.getDataElement().getElement() instanceof LayerSymbol){
            if(((LayerSymbol) (this.getDataElement().getElement())).getDeclaration() instanceof CustomLayerDeclaration){
                templatePath = relativePath + "CustomLayer" + FTL_FILE_ENDING;
            }
        }
        getTemplateConfiguration().processTemplate(ftlContext, templatePath, writer);
    }

    public void include(VariableSymbol element, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(element);

        if (element.isAtomic()){
            if (element.getType() == VariableSymbol.Type.IO) {
                if (element.isInput()){
                    include(TEMPLATE_ELEMENTS_DIR_PATH, "Input", writer, netDefinitionMode);
                }
                else {
                    include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer, netDefinitionMode);
                }
            }
            else if (element.getType() == VariableSymbol.Type.LAYER) {
                if (element.getMember() == VariableSymbol.Member.STATE) {
                    include(TEMPLATE_ELEMENTS_DIR_PATH, "Output", writer, netDefinitionMode);
                } else if (element.getMember() == VariableSymbol.Member.NONE) {
                    include(TEMPLATE_ELEMENTS_DIR_PATH, element.getLayerVariableDeclaration().getLayer().getName(), writer, netDefinitionMode);
                }
            }
        }
        else {
            include((ArchitectureElementSymbol) element.getResolvedThis().get(), writer, netDefinitionMode);
        }

        setCurrentElement(previousElement);
    }

    public void include(LayerSymbol layer, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(layer);

        if(layer.isAtomic()){
            String templateName = layer.getDeclaration().getName();
            include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
        }else{
            include((ArchitectureElementSymbol) layer.getResolvedThis().get(), writer, netDefinitionMode);
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
        }
        else if (architectureElement instanceof LayerSymbol){
            include((LayerSymbol) architectureElement, writer, netDefinitionMode);
        }
        else if (architectureElement instanceof ConstantSymbol) {

        }
        else {
            include((VariableSymbol) architectureElement, writer, netDefinitionMode);
        }
    }

    public void include(ArchitectureElementSymbol architectureElementSymbol, String netDefinitionMode) {
        include(architectureElementSymbol, NetDefinitionMode.fromString(netDefinitionMode));
    }

    public void include(ArchitectureElementSymbol architectureElement, NetDefinitionMode netDefinitionMode){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, getWriter(), netDefinitionMode);
    }

    public Set<String> getStreamInputNames(SerialCompositeElementSymbol stream, boolean outputAsArray) {
        return getStreamInputs(stream, outputAsArray).keySet();
    }

    public Map<String, List<String>> getStreamInputs(SerialCompositeElementSymbol stream, boolean outputAsArray) {
        Map<String, List<String>> inputs = new LinkedHashMap<>();

        for (ArchitectureElementSymbol element : stream.getFirstAtomicElements()) {
            if (element.isInput() || element.isOutput()) {
                List<Integer> intDimensions = element.getOutputTypes().get(0).getDimensions();

                List<String> dimensions = new ArrayList<>();
                for (Integer intDimension : intDimensions) {
                    dimensions.add(intDimension.toString());
                }

                String name = getName(element);

                if (outputAsArray && element.isOutput() && element instanceof VariableSymbol) {
                    VariableSymbol variable = (VariableSymbol) element;

                    if (variable.getType() == VariableSymbol.Type.IO) {
                        name = getNameAsArray(name);
                    }
                }

                inputs.put(name, dimensions);
            }
            else if (element instanceof ConstantSymbol) {
                inputs.put(getName(element), Arrays.asList("1"));
            }
        }

        inputs.putAll(getStreamLayerVariableMembers(stream, false));
        return inputs;
    }

    public String getNameAsArray(String name) {
        return name.replaceAll("([0-9]+)_$", "[$1]");
    }

    private Map<String, List<String>> getStreamLayerVariableMembers(SerialCompositeElementSymbol stream, boolean includeOutput) {
        Map<String, List<String>> members = new LinkedHashMap<>();

        List<ArchitectureElementSymbol> elements = stream.getSpannedScope().resolveLocally(ArchitectureElementSymbol.KIND);
        for (ArchitectureElementSymbol element : elements) {
            if (element instanceof VariableSymbol) {
                VariableSymbol variable = (VariableSymbol) element;

                if (variable.getType() == VariableSymbol.Type.LAYER && (variable.getMember() == VariableSymbol.Member.NONE)) {
                    LayerVariableDeclarationSymbol layerVariableDeclaration = variable.getLayerVariableDeclaration();

                    if (layerVariableDeclaration.getLayer().getDeclaration().isPredefined()) {
                        PredefinedLayerDeclaration predefinedLayerDeclaration =
                                (PredefinedLayerDeclaration) layerVariableDeclaration.getLayer().getDeclaration();

                        int arrayLength = predefinedLayerDeclaration.getArrayLength(VariableSymbol.Member.STATE);

                        for (int i = 0; i < arrayLength; ++i) {
                            String name = variable.getName() + "_state_";

                            if (arrayLength > 1) {
                                name += i + "_";
                            }

                            List<Integer> intDimensions = predefinedLayerDeclaration.computeOutputTypes(
                                    layerVariableDeclaration.getLayer().getInputTypes(),
                                    layerVariableDeclaration.getLayer(),
                                    VariableSymbol.Member.STATE
                            ).get(0).getDimensions();

                            List<String> dimensions = new ArrayList<>();

                            for (Integer intDimension : intDimensions) {
                                dimensions.add(intDimension.toString());
                            }

                            members.put(name, dimensions);
                        }

                        if (includeOutput) {
                            arrayLength = predefinedLayerDeclaration.getArrayLength(VariableSymbol.Member.OUTPUT);

                            for (int i = 0; i < arrayLength; ++i) {
                                String name = variable.getName() + "_output_";

                                if (arrayLength > 1) {
                                    name += i + "_";
                                }

                                List<Integer> intDimensions = predefinedLayerDeclaration.computeOutputTypes(
                                        layerVariableDeclaration.getLayer().getInputTypes(),
                                        layerVariableDeclaration.getLayer(),
                                        VariableSymbol.Member.OUTPUT
                                ).get(0).getDimensions();

                                List<String> dimensions = new ArrayList<>();

                                for (Integer intDimension : intDimensions) {
                                    dimensions.add(intDimension.toString());
                                }

                                members.put(name, dimensions);
                            }
                        }
                    }
                }
            }
        }
        return members;
    }

    public Set<String> getStreamOutputNames(SerialCompositeElementSymbol stream, boolean asArray) {
        Set<String> outputNames = new LinkedHashSet<>();

        for (ArchitectureElementSymbol element : stream.getLastAtomicElements()) {
            if (element.isOutput()) {
                String name = getName(element);

                if (asArray && element instanceof VariableSymbol) {
                    VariableSymbol variable = (VariableSymbol) element;

                    if (variable.getType() == VariableSymbol.Type.IO) {
                        name = getNameAsArray(name);
                    }
                }

                outputNames.add(name);
            }
        }

        outputNames.addAll(getStreamLayerVariableMembers(stream, true).keySet());


        return outputNames;
    }

    public List<String> cutDimensions(List<String> dimensions) {
        while (dimensions.size() > 1 && dimensions.get(dimensions.size() - 1).equals("1")) {
            dimensions.remove(dimensions.size() - 1);
        }

        return dimensions;
    }

    public List<Integer> cutDimensionsInteger(List<Integer> dimensions) {
        while (dimensions.size() > 1 && dimensions.get(dimensions.size() - 1).equals(1)) {
            dimensions.remove(dimensions.size() - 1);
        }

        return dimensions;
    }

}
