/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement;

import de.monticore.generating.templateengine.TemplateController;
import de.monticore.lang.monticar.cnnarch.generator.ArchitectureElementData;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchTemplateController;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch.gluongenerator.AllAttentionModels;
import de.monticore.lang.monticar.cnnarch.gluongenerator.NetDefinitionMode;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.lang.monticar.types2._ast.ASTElementType;

import java.io.Writer;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class CNNArch2GluonTemplateController extends CNNArchTemplateController {
    public static final String NET_DEFINITION_MODE_KEY = "mode";
    public List<String> worked_list = new ArrayList<String>();
    public CNNArch2GluonTemplateController(ArchitectureSymbol architecture,
                                           TemplateConfiguration templateConfiguration) {
        super(architecture, templateConfiguration);
    }
    public String getDefinedOutputDimension(){
        // function calculates the output shape as defined in the .emadl, used for AdaNet layer
        ArchTypeSymbol types = ((IODeclarationSymbol)this.getArchitecture().getOutputs().get(0).getDeclaration()).getType();
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append("(");
        types.getDimensions().forEach(elem->stringBuilder.append(elem).append(','));
        stringBuilder.append(")");

        return stringBuilder.toString();
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
        if(layer.getName().equals(AllPredefinedLayers.AdaNet_Name)&& netDefinitionMode.equals(NetDefinitionMode.ADANET_CONSTRUCTION)){
            // construct the AdaNet Layer
            include(TEMPLATE_ELEMENTS_DIR_PATH,"AdaNet",writer,netDefinitionMode);
        }else if(layer.isAtomic()){
            String templateName = layer.getDeclaration().getName();
            include(TEMPLATE_ELEMENTS_DIR_PATH, templateName, writer, netDefinitionMode);
        }else if(layer.isArtificial() && this.containsAdaNet()){
            if(netDefinitionMode.equals(NetDefinitionMode.ARTIFICIAL_ARCH_CLASS)){
                boolean originalArtificialState = layer.isArtificial();
                layer.setArtificial(false);
                if (!this.worked_list.contains(layer.getName())){
                    include(TEMPLATE_ELEMENTS_DIR_PATH, "ArtificialArchClass", writer, netDefinitionMode);
                    this.worked_list.add(layer.getName());
                }
                layer.setArtificial(originalArtificialState);
            }else{
                include((ArchitectureElementSymbol) layer.getResolvedThis().get(), writer, netDefinitionMode);
            }
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

    public void include(SerialCompositeElementSymbol compositeElement, Integer episodicSubNetIndex, Writer writer, NetDefinitionMode netDefinitionMode){
        ArchitectureElementData previousElement = getCurrentElement();
        setCurrentElement(compositeElement);

        for (ArchitectureElementSymbol element : compositeElement.getEpisodicSubNetworks().get(episodicSubNetIndex)){
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

    public void include(ArchitectureElementSymbol architectureElementSymbol, Integer episodicSubNetIndex, String netDefinitionMode) {
        include(architectureElementSymbol, episodicSubNetIndex, NetDefinitionMode.fromString(netDefinitionMode));
    }

    public void include(ArchitectureElementSymbol architectureElement, NetDefinitionMode netDefinitionMode){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include(architectureElement, getWriter(), netDefinitionMode);
    }

    public void include(ArchitectureElementSymbol architectureElement, Integer episodicSubNetIndex, NetDefinitionMode netDefinitionMode){
        if (getWriter() == null){
            throw new IllegalStateException("missing writer");
        }
        include((SerialCompositeElementSymbol) architectureElement, episodicSubNetIndex, getWriter(), netDefinitionMode);
    }

    public Set<String> getStreamInputNames(SerialCompositeElementSymbol stream, boolean outputAsArray) {
        return getStreamInputs(stream, outputAsArray).keySet();
    }

    public Set<String> getSubnetInputNames(List<ArchitectureElementSymbol> subNet) {
        return getSubnetInputs(subNet).keySet();
    }

    public ArrayList<String> getStreamInputVariableNames(SerialCompositeElementSymbol stream, boolean outputAsArray) {
        ArrayList<String> inputVariableNames = new ArrayList<String>();
        for (ArchitectureElementSymbol element : stream.getFirstAtomicElements()) {
            if (element.isInput()) {
                inputVariableNames.add(getName(element));
            }
        }
        return inputVariableNames;
    }

    public List<String> get(Map<String, List<String>> map, String name) {
        return map.get(name);
    }

    public List<String> getUnrollInputNames(UnrollInstructionSymbol unroll, String variable) {
        List<String> inputNames = new LinkedList<>(getStreamInputNames(unroll.getBody(), true));
        Map<String, String> pairs = getUnrollPairs(unroll.getBody(), unroll.getResolvedBodies().get(0), variable);

        for (int i = 0; i != inputNames.size(); ++i) {
            if (pairs.containsKey(inputNames.get(i))) {
                inputNames.set(i, pairs.get(inputNames.get(i)));
            }
        }

        return inputNames;
    }

    public Collection<List<String>> getStreamInputDimensions(SerialCompositeElementSymbol stream) {
        return getStreamInputs(stream, false).values();
    }

    public Collection<List<String>> getStreamOutputDimensions(SerialCompositeElementSymbol stream) {
        return getStreamOutputs(stream, false).values();
    }

    public ArrayList<String> getStreamOutputVariableNames(SerialCompositeElementSymbol stream, boolean outputAsArray) {
        ArrayList<String> outputVariableNames = new ArrayList<String>();
        for (ArchitectureElementSymbol element : stream.getLastAtomicElements()) {
            if (element.isOutput()) {
                outputVariableNames.add(getName(element));
            }
        }
        return outputVariableNames;
    }

    public Collection<List<String>> getStreamInputInformation(SerialCompositeElementSymbol stream) {
        Map<String, List<String>> dimensions = getStreamInputs(stream, false);
        Map<String, List<String>> domains = getStreamInputDomains(stream);
        Collection<List<String>> information = new HashSet<List<String>>();
        for (String name : dimensions.keySet()) {
            List<String> newEntry = new ArrayList<String>();
            newEntry.add(name);
        }
        return null;
    }


    /*public Collection<List<String>> getStreamOutputsWithTypes(SerialCompositeElementSymbol stream) {

    }*/

    public String getOutputName() {
        return getNameWithoutIndex(getName(getArchitectureOutputSymbols().get(0)));
    }

    public String getNameAsArray(String name) {
        return name.replaceAll("([0-9]+)_$", "[$1]");
    }

    public String getNameWithoutIndex(String name) {
        return name.replaceAll("([0-9]+)_$", "").replaceAll("\\[[^\\]]+\\]$", "");
    }

    public String getIndex(String name, boolean defaultToZero) {
        Pattern pattern = Pattern.compile("\\[([^\\]]+)\\]$");
        Matcher matcher = pattern.matcher(name);

        if (matcher.find()) {
            return matcher.group(1);
        }

        return defaultToZero ? "0" : "";
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

    public Set<String> getSubnetOutputNames(List<ArchitectureElementSymbol> subNet){
        Set<String> outputNames = new LinkedHashSet<>();

        for (ArchitectureElementSymbol element : subNet.get(subNet.size()-1).getLastAtomicElements()) {
            String name = getName(element);

            outputNames.add(name);
        }

        return outputNames;
    }

    public int getSubnetOutputSize(List<ArchitectureElementSymbol> subNet){
        int outputSize = 0;
        for (ArchitectureElementSymbol element : subNet.get(subNet.size()-1).getLastAtomicElements()) {
            outputSize += element.getOutputTypes().size();
        }
        if(outputSize == 0){
            outputSize = 1;
        }
        return outputSize;
    }

    public List<String> getUnrollOutputNames(UnrollInstructionSymbol unroll, String variable) {
        List<String> outputNames = new LinkedList<>(getStreamOutputNames(unroll.getBody(), true));
        Map<String, String> pairs = getUnrollPairs(unroll.getBody(), unroll.getResolvedBodies().get(0), variable);

        for (int i = 0; i != outputNames.size(); ++i) {
            if (pairs.containsKey(outputNames.get(i))) {
                outputNames.set(i, pairs.get(outputNames.get(i)));
            }
        }

        return outputNames;
    }

    public boolean endsWithArgmax(SerialCompositeElementSymbol stream) {
        List<ArchitectureElementSymbol> elements = stream.getElements();

        if (elements.size() > 1) {
            // Check second last element because last element is output
            ArchitectureElementSymbol secondLastElement = elements.get(elements.size() - 2);

            return secondLastElement.getName().equals(AllPredefinedLayers.ARG_MAX_NAME);
        }

        return false;
    }

    // Used to initialize all layer variable members which are passed through the networks
    public  Map<String, List<String>> getLayerVariableMembers() {
        Map<String, List<String>> members = new LinkedHashMap<>();

        for (NetworkInstructionSymbol networkInstruction : getArchitecture().getNetworkInstructions()) {
            members.putAll(getStreamLayerVariableMembers(networkInstruction.getBody(), true));
        }

        return members;
    }

    // Calculate differently named VariableSymbol elements in two streams, currently used for the UnrollInstructionSymbol
    // body which is resolved with t = CONST_OFFSET and the current body of the actual timestep t
    public Map<String, String> getUnrollPairs(ArchitectureElementSymbol element, ArchitectureElementSymbol current, String variable) {
        Map<String, String> pairs = new HashMap<>();

        if (element instanceof CompositeElementSymbol && current instanceof CompositeElementSymbol) {
            List<ArchitectureElementSymbol> elements = ((CompositeElementSymbol) element).getElements();
            List<ArchitectureElementSymbol> currentElements = ((CompositeElementSymbol) current).getElements();

            if (elements.size() == currentElements.size()) {
                for (int i = 0; i != currentElements.size(); ++i) {
                    String name = getName(elements.get(i));
                    String currentName = getName(currentElements.get(i));

                    if (elements.get(i).isOutput()) {
                        name = getNameAsArray(name);
                    }

                    if (currentElements.get(i).isOutput()) {
                        currentName = getNameAsArray(currentName);
                    }

                    if (elements.get(i) instanceof VariableSymbol && currentElements.get(i) instanceof VariableSymbol) {
                        if (name != null && currentName != null && !name.equals(currentName)) {
                            String newIndex = variable + "-1+" + getIndex(currentName, true);
                            currentName = currentName.replaceAll("\\[([0-9]+)\\]$", "[" + newIndex + "]");

                            pairs.put(name, currentName);
                        }
                    }

                    pairs.putAll(getUnrollPairs(elements.get(i), currentElements.get(i), variable));
                }
            }
        }

        return pairs;
    }

    public Map<String, List<String>> getStreamInputDomains(SerialCompositeElementSymbol stream) {

        Map<String, List<String>> inputTypes = new LinkedHashMap<>();
        for (ArchitectureElementSymbol element : stream.getFirstAtomicElements()) {
            if (element.isInput() || element.isOutput()) {
                ASTElementType type = element.getOutputTypes().get(0).getDomain();
                HashMap<String,String> ranges = element.getOutputTypes().get(0).getElementRange();
                if (ranges.get("min") == "-inf")
                    ranges.put("min", "float('-inf')");
                if (ranges.get("max") == "inf")
                    ranges.put("max", "float('inf')");

                String typeAsString = new String();

                if(type.isBoolean())
                    typeAsString = "bool";
                else if (type.isComplex())
                    typeAsString = "complex";
                else if (type.isNaturalNumber() || type.isWholeNumber())
                    typeAsString = "int";
                else if (type.isRational())
                    typeAsString = "float";

                String name = getName(element);
                ArrayList<String> domain = new ArrayList<String>();
                domain.add(typeAsString);
                domain.add(ranges.get("min"));
                domain.add(ranges.get("max"));
                inputTypes.put(name, domain);
            }
        }
        return inputTypes;
    }

    public Map<String, List<String>> getStreamOutputDomains(SerialCompositeElementSymbol stream) {

        Map<String, List<String>> outputTypes = new LinkedHashMap<>();
        for (ArchitectureElementSymbol element : stream.getLastAtomicElements()) {
            if (element.isInput() || element.isOutput()) {
                ASTElementType type = element.getInputTypes().get(0).getDomain();
                HashMap<String,String> ranges = element.getInputTypes().get(0).getElementRange();
                if (ranges.get("min") == "-inf")
                    ranges.put("min", "float('-inf')");
                if (ranges.get("max") == "inf")
                    ranges.put("max", "float('inf')");

                String typeAsString = new String();

                if(type.isBoolean())
                    typeAsString = "bool";
                else if (type.isComplex())
                    typeAsString = "complex";
                else if (type.isNaturalNumber() || type.isWholeNumber())
                    typeAsString = "int";
                else if (type.isRational())
                    typeAsString = "float";

                String name = getName(element);
                ArrayList<String> domain = new ArrayList<String>();
                domain.add(typeAsString);
                domain.add(ranges.get("min"));
                domain.add(ranges.get("max"));
                outputTypes.put(name, domain);
            }
        }
        return outputTypes;
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

    public Map<String, List<String>> getSubnetInputs(List<ArchitectureElementSymbol> subNet) {
        Map<String, List<String>> inputs = new LinkedHashMap<>();

        for (ArchitectureElementSymbol element : subNet.get(0).getFirstAtomicElements()) {

            if (element instanceof ConstantSymbol) {
                inputs.put(getName(element), Arrays.asList("1"));
            }
            else {
                List<Integer> intDimensions = element.getOutputTypes().get(0).getDimensions();

                List<String> dimensions = new ArrayList<>();
                for (Integer intDimension : intDimensions) {
                    dimensions.add(intDimension.toString());
                }

                String name = getName(element);

                inputs.put(name, dimensions);
            }
        }

        return inputs;
    }

    public Map<String, List<String>> getStreamOutputs(SerialCompositeElementSymbol stream, boolean outputAsArray) {
        Map<String, List<String>> outputs = new LinkedHashMap<>();

        for (ArchitectureElementSymbol element : stream.getLastAtomicElements()) {
            if (element.isInput() || element.isOutput()) {
                List<Integer> intDimensions = element.getPrevious().get(0).getOutputTypes().get(0).getDimensions();

                List<String> dimensions = new ArrayList<>();
                for (Integer intDimension : intDimensions) {
                    dimensions.add(intDimension.toString());
                }

                if (dimensions.isEmpty())
                    dimensions.add("unknown");

                String name = getName(element);

                if (outputAsArray && element.isOutput() && element instanceof VariableSymbol) {
                    VariableSymbol variable = (VariableSymbol) element;

                    if (variable.getType() == VariableSymbol.Type.IO) {
                        name = getNameAsArray(name);
                    }
                }
                outputs.put(name, dimensions);
            }
            else if (element instanceof ConstantSymbol) {
                outputs.put(getName(element), Arrays.asList("1"));
            }
        }

        outputs.putAll(getStreamLayerVariableMembers(stream, false));

        return outputs;
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

    // cuts
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


    public boolean hasUnrollInstructions() {
        for (NetworkInstructionSymbol networkInstruction : getArchitecture().getNetworkInstructions()) {
            if (networkInstruction.isUnroll()) {
                return true;
            }
        }

        return false;
    }

    public boolean isAttentionNetwork(){
        return AllAttentionModels.getAttentionModels().contains(getComponentName());
    }

    public boolean isArchitectureOutput(String element){
        return getArchitectureOutputs().contains(element.replace("1000000", "0"));
    }

    public int getBeamSearchMaxLength(UnrollInstructionSymbol unroll){
        return unroll.getIntValue(AllPredefinedLayers.MAX_LENGTH_NAME).get();
    }

    public int getBeamSearchWidth(UnrollInstructionSymbol unroll){
        // Beam search with width 1 is greedy search
        return unroll.getIntValue(AllPredefinedLayers.WIDTH_NAME).orElse(1);
    }

}
