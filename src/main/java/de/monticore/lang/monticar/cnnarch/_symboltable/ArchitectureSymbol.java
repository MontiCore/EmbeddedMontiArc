/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
/* generated by template symboltable.ScopeSpanningSymbol*/


package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.cnnarch._cocos.CheckLayerPathParameter;
import de.monticore.lang.monticar.cnnarch.helper.Utils;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

public class ArchitectureSymbol extends CommonScopeSpanningSymbol {

    public static final ArchitectureKind KIND = new ArchitectureKind();

    private List<LayerVariableDeclarationSymbol> layerVariableDeclarations = new ArrayList<>();
    private List<NetworkInstructionSymbol> networkInstructions = new ArrayList<>();
    private List<VariableSymbol> inputs = new ArrayList<>();
    private List<VariableSymbol> outputs = new ArrayList<>();
    private List<ConstantSymbol> constants = new ArrayList<>();
    private String dataPath;
    private String weightsPath;
    private String componentName;
    private ArchitectureSymbol auxiliaryArchitecture;
    private boolean AdaNet = false;
    //attribute for the path for custom python files
    private String customPyFilesPath;
    private boolean useDgl;
    // path to the AdaNet python files, can be changed EMADL2CPP by changing the value in the EMADL2CPP generator
    private String adaNetUtils = "./src/main/resources/AdaNet/";
    public ArchitectureSymbol() {
        super("", KIND);
    }

    public List<LayerVariableDeclarationSymbol> getLayerVariableDeclarations() {
        return layerVariableDeclarations;
    }

    public void setLayerVariableDeclarations(List<LayerVariableDeclarationSymbol> layerVariableDeclarations) {
        this.layerVariableDeclarations = layerVariableDeclarations;
    }
    public void setAuxiliaryArchitecture(ArchitectureSymbol auxiliaryArchitecture){
        this.auxiliaryArchitecture = auxiliaryArchitecture;
    }

    public ArchitectureSymbol getAuxiliaryArchitecture(){
        return this.auxiliaryArchitecture;
    }

    public String getAdaNetUtils(){return this.adaNetUtils;}
    public void setAdaNetUtils(String adaNetUtils){this.adaNetUtils=adaNetUtils;}
    public boolean containsAdaNet() {
        return this.AdaNet;
    }

    public List<NetworkInstructionSymbol> getNetworkInstructions() {
        return networkInstructions;
    }

    public void setNetworkInstructions(List<NetworkInstructionSymbol> networkInstructions) {
        this.networkInstructions = networkInstructions;
        // if one of the network instructions contains an AdaNet layer the AdaNet flag is set to true
        for (NetworkInstructionSymbol networkInstruction : networkInstructions) {
            this.AdaNet = this.AdaNet | networkInstruction.containsAdaNet();
        }
    }

    public List<SerialCompositeElementSymbol> getStreams() {
        List<SerialCompositeElementSymbol> streams = new ArrayList<>();

        for (NetworkInstructionSymbol networkInstruction : getNetworkInstructions()) {
            if (networkInstruction.isStream()) {
                streams.add(networkInstruction.getBody());
            }
        }

        return streams;
    }

    public String getDataPath() {
        return this.dataPath;
    }

    public void setDataPath(String dataPath) {
        this.dataPath = dataPath;
    }

    public String getWeightsPath() {
        return this.weightsPath;
    }

    public void setWeightsPath(String weightsPath) {
        this.weightsPath = weightsPath;
    }

    public void setComponentName(String componentName){
        this.componentName = componentName;
    }

    public String getComponentName(){
        return this.componentName;
    }

    public List<VariableSymbol> getInputs() {
        return inputs;
    }

    public void setInputs(List<VariableSymbol> inputs) {
        this.inputs = inputs;
    }

    public List<VariableSymbol> getOutputs() {
        return outputs;
    }

    public void setOutputs(List<VariableSymbol> outputs) {
        this.outputs = outputs;
    }

    public List<ConstantSymbol> getConstants() {
        return constants;
    }

    public Collection<IODeclarationSymbol> getIODeclarations(){
        return getEnclosingScope().resolveLocally(IODeclarationSymbol.KIND);
    }

    public Collection<LayerDeclarationSymbol> getLayerDeclarations(){
        return getSpannedScope().resolveLocally(LayerDeclarationSymbol.KIND);
    }

    public Collection<UnrollDeclarationSymbol> getUnrollDeclarations(){
        return getSpannedScope().resolveLocally(UnrollDeclarationSymbol.KIND);
    }

    public void setCustomPyFilesPath(String customPyFilesPath) { this.customPyFilesPath = customPyFilesPath; }

    public String getCustomPyFilesPath() { return customPyFilesPath; }

    public void setUseDgl(boolean useDgl) { this.useDgl = useDgl; }

    public boolean getUseDgl() { return useDgl; }

    public void resolve() {
        for (NetworkInstructionSymbol networkInstruction : getNetworkInstructions()) {
            networkInstruction.checkIfResolvable();

            try {
                networkInstruction.resolveOrError();
            }
            catch (ArchResolveException e) {
                // Do nothing; error is already logged
            }
        }
    }

    public boolean isResolved(){
        boolean resolved = true;

        for (NetworkInstructionSymbol networkInstruction : getNetworkInstructions()) {
            resolved &= networkInstruction.isResolved();
        }

        return resolved;
    }

    public boolean isResolvable(){
        boolean resolvable = true;

        for (NetworkInstructionSymbol networkInstruction : getNetworkInstructions()) {
            resolvable &= networkInstruction.isResolvable();
        }

        return resolvable;
    }

    public void putInScope(Scope scope){
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)){
            scope.getAsMutableScope().add(this);
            Utils.recursiveSetResolvingFilters(getSpannedScope(), scope.getResolvingFilters());
        }
    }

    /*
      Creates a unresolved copy of this architecture and
      adds the copy to the scope given as argument.
      Useful to create instances.
      This works even if "this" is already resolved.
     */
    public ArchitectureSymbol preResolveDeepCopy(Scope enclosingScopeOfCopy){
        ArchitectureSymbol copy = new ArchitectureSymbol();

        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        copy.getSpannedScope().getAsMutableScope().add(AllPredefinedVariables.createTrueConstant());
        copy.getSpannedScope().getAsMutableScope().add(AllPredefinedVariables.createFalseConstant());

        for (LayerDeclarationSymbol layerDeclaration : AllPredefinedLayers.createList()){
            copy.getSpannedScope().getAsMutableScope().add(layerDeclaration);
        }

        for (UnrollDeclarationSymbol unrollDeclaration : AllPredefinedLayers.createUnrollList()){
            copy.getSpannedScope().getAsMutableScope().add(unrollDeclaration);
        }


        for (LayerDeclarationSymbol layerDeclaration : getLayerDeclarations()){
            if (!layerDeclaration.isPredefined()) {
                copy.getSpannedScope().getAsMutableScope().add(layerDeclaration.deepCopy());
            }
        }

        List<LayerVariableDeclarationSymbol> copyLayerVariableDeclarations = new ArrayList<>();
        for (LayerVariableDeclarationSymbol layerVariableDeclaration : getLayerVariableDeclarations()) {
            LayerVariableDeclarationSymbol copyLayerVariableDeclaration =
                    (LayerVariableDeclarationSymbol) layerVariableDeclaration.preResolveDeepCopy();
            copyLayerVariableDeclaration.putInScope(copy.getSpannedScope());
            copyLayerVariableDeclarations.add(copyLayerVariableDeclaration);
        }
        copy.setLayerVariableDeclarations(copyLayerVariableDeclarations);

        List<NetworkInstructionSymbol> copyNetworkInstructions = new ArrayList<>();
        for (NetworkInstructionSymbol networkInstruction : getNetworkInstructions()) {
            NetworkInstructionSymbol copyNetworkInstruction = (NetworkInstructionSymbol) networkInstruction.preResolveDeepCopy();
            copyNetworkInstruction.putInScope(copy.getSpannedScope());
            copyNetworkInstructions.add(copyNetworkInstruction);
        }
        copy.setNetworkInstructions(copyNetworkInstructions);

        copy.putInScope(enclosingScopeOfCopy);
        return copy;
    }

    public void processLayerPathParameterTags(HashMap layerPathParameterTags){
        for(NetworkInstructionSymbol networkInstruction : networkInstructions){
            List<ArchitectureElementSymbol> elements = networkInstruction.getBody().getElements();
            processElementsLayerPathParameterTags(elements, layerPathParameterTags);
        }
    }

    public void processElementsLayerPathParameterTags(List<ArchitectureElementSymbol> elements, HashMap layerPathParameterTags){
        for (ArchitectureElementSymbol element : elements){
            if (element instanceof SerialCompositeElementSymbol || element instanceof ParallelCompositeElementSymbol){
                processElementsLayerPathParameterTags(((CompositeElementSymbol) element).getElements(), layerPathParameterTags);
            }else if (element instanceof LayerSymbol){
                for (ArgumentSymbol param : ((LayerSymbol) element).getArguments()){
                    boolean isPathParam = false;
                    if (param.getParameter() != null) {
                        for (Constraints constr : param.getParameter().getConstraints()) {
                            if (constr.name().equals("PATH_TAG_OR_PATH")) {
                                isPathParam = true;
                            }
                        }
                    }
                    if (isPathParam){
                        String paramValue = param.getRhs().getStringValue().get();
                        if (paramValue.startsWith("tag:")) {
                            String pathTag = param.getRhs().getStringValue().get().split(":")[1];
                            String path = (String) layerPathParameterTags.get(pathTag);
                            param.setRhs(ArchSimpleExpressionSymbol.of(path));
                            CheckLayerPathParameter.check((LayerSymbol) element, path, pathTag, layerPathParameterTags);
                        }else{
                            CheckLayerPathParameter.check((LayerSymbol) element, paramValue, "", layerPathParameterTags);
                        }
                    }
                }
            }
        }
    }

    public void processForEpisodicReplayMemory(){
        for(NetworkInstructionSymbol networkInstruction : networkInstructions){
            List<ArchitectureElementSymbol> elements = networkInstruction.getBody().getElements();
            List<ArchitectureElementSymbol> elementsNew = new ArrayList<>();
            List<List<ArchitectureElementSymbol>> episodicSubNetworks = new ArrayList<>(new ArrayList<>());
            List<ArchitectureElementSymbol> currentEpisodicSubNetworkElements = new ArrayList<>();
            boolean anyEpisodicLocalAdaptation = false;

            for (ArchitectureElementSymbol element : elements){
                if (AllPredefinedLayers.EPISODIC_REPLAY_LAYER_NAMES.contains(element.getName())) {
                    boolean use_replay = false;
                    boolean use_local_adaptation = false;
                    boolean use_replay_specified = false;
                    boolean use_local_adaptation_specified = false;

                    for (ArgumentSymbol arg : ((LayerSymbol)element).getArguments()){
                        if (arg.getName().equals(AllPredefinedLayers.USE_REPLAY_NAME)){
                                use_replay_specified = true;
                                if ((boolean)arg.getRhs().getValue().get()) {
                                    use_replay = true;
                                    break;
                                }
                        }else if (arg.getName().equals(AllPredefinedLayers.USE_LOCAL_ADAPTATION_NAME)){
                            use_local_adaptation_specified = true;
                            if ((boolean)arg.getRhs().getValue().get()) {
                                use_local_adaptation = true;
                                anyEpisodicLocalAdaptation = true;
                                break;
                            }
                        }
                    }

                    if (!use_replay_specified) {
                        for (ParameterSymbol param : ((LayerSymbol) element).getDeclaration().getParameters()) {
                            if (param.getName().equals(AllPredefinedLayers.USE_REPLAY_NAME) &&
                                    (boolean) param.getDefaultExpression().get().getValue().get()) {
                                use_replay = true;
                                break;
                            }
                        }
                    }
                    if (!use_local_adaptation_specified) {
                        for (ParameterSymbol param : ((LayerSymbol) element).getDeclaration().getParameters()) {
                            if (param.getName().equals(AllPredefinedLayers.USE_LOCAL_ADAPTATION_NAME) &&
                                    (boolean) param.getDefaultExpression().get().getValue().get()) {
                                use_local_adaptation = true;
                                anyEpisodicLocalAdaptation = true;
                                break;
                            }
                        }
                    }

                    if (use_replay || use_local_adaptation){
                        if (!currentEpisodicSubNetworkElements.isEmpty()){
                            episodicSubNetworks.add(currentEpisodicSubNetworkElements);
                        }
                        currentEpisodicSubNetworkElements = new ArrayList<>();
                    }
                }
                currentEpisodicSubNetworkElements.add(element);
            }
            if (!currentEpisodicSubNetworkElements.isEmpty() && !episodicSubNetworks.isEmpty()){
                episodicSubNetworks.add(currentEpisodicSubNetworkElements);
            }
                networkInstruction.getBody().setEpisodicSubNetworks(episodicSubNetworks);
                networkInstruction.getBody().setAnyEpisodicLocalAdaptation(anyEpisodicLocalAdaptation);
        }
    }
}
