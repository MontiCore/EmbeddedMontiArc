/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.helper.Utils;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedLayers;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class VariableSymbol extends ArchitectureElementSymbol {

    public enum Type {
        IO,
        LAYER,
        UNKNOWN
    }

    public enum Member {
        STATE,
        OUTPUT,
        NONE,
        UNKNOWN;

        public static Member fromString(final String member) {
            switch (member) {
                case "state":
                    return STATE;
                case "output":
                    return OUTPUT;
                case "":
                    return NONE;
                default:
                    return UNKNOWN;
            }
        }
    }

    private Type type = Type.UNKNOWN;

    private Member member = Member.NONE;
    private ArchSimpleExpressionSymbol arrayAccess = null;

    private VariableDeclarationSymbol declaration;

    protected VariableSymbol(String name) {
        super(name);
    }

    public Type getType() {
        if (type == Type.UNKNOWN) {
            getDeclaration();
        }

        return type;
    }

    public Member getMember() {
        return member;
    }

    protected void setMember(Member member) {
        this.member = member;
    }

    protected void setMember(String member) {
        this.member = Member.fromString(member);
    }

    public Optional<ArchSimpleExpressionSymbol> getArrayAccess() {
        return Optional.ofNullable(arrayAccess);
    }

    protected void setArrayAccess(ArchSimpleExpressionSymbol arrayAccess) {
        this.arrayAccess = arrayAccess;
    }

    protected void setArrayAccess(int arrayAccess) {
        this.arrayAccess = ArchSimpleExpressionSymbol.of(arrayAccess);
        this.arrayAccess.putInScope(getSpannedScope());
    }

    public List<ArchTypeSymbol> getInputTypes() {
        if (getType() == Type.LAYER && getMember() != Member.STATE) {
            return getLayerVariableDeclaration().getLayer().getInputTypes();
        }
        else {
            return super.getInputTypes();
        }
    }

    public VariableDeclarationSymbol getDeclaration() {
        if (declaration == null) {
            Collection<VariableDeclarationSymbol> collection
                    = getArchitecture().getSpannedScope().resolveMany(getName(), VariableDeclarationSymbol.KIND);

            if (!collection.isEmpty()) {
                declaration = collection.iterator().next();

                if (declaration instanceof IODeclarationSymbol) {
                    type = Type.IO;
                }
                else if (declaration instanceof LayerVariableDeclarationSymbol) {
                    type = Type.LAYER;
                }
            }
            else {
                throw new IllegalStateException("No variable declaration found");
            }
        }

        return declaration;
    }

    // Only call when type == VariableType.IO
    public IODeclarationSymbol getIoDeclaration() {
        return (IODeclarationSymbol) getDeclaration();
    }

    // Only call when type == VariableType.LAYER
    public LayerVariableDeclarationSymbol getLayerVariableDeclaration() {
        return (LayerVariableDeclarationSymbol) getDeclaration();
    }

    @Override
    public boolean isResolvable() {
        return super.isResolvable() && getDeclaration() != null;
    }

    @Override
    public boolean isInput() {
        if (getType() == Type.IO) {
            return getIoDeclaration().isInput();
        }
        else if (getType() == Type.LAYER) {
            return getMember() == Member.STATE || getMember() == Member.OUTPUT;
        }

        return false;
    }

    @Override
    public boolean isOutput() {
        if (getType() == Type.IO) {
            return getIoDeclaration().isOutput();
        }
        else if (getType() == Type.LAYER) {
            return getMember() == Member.STATE || getMember() == Member.NONE;
        }

        return false;
    }

    @Override
    public boolean isAtomic() {
        return getResolvedThis().isPresent() && getResolvedThis().get() == this;
    }

    @Override
    public List<ArchitectureElementSymbol> getFirstAtomicElements() {
        if (getResolvedThis().isPresent() && getResolvedThis().get() != this) {
            return ((ArchitectureElementSymbol) getResolvedThis().get()).getFirstAtomicElements();
        }
        else {
            return Collections.singletonList(this);
        }
    }

    @Override
    public List<ArchitectureElementSymbol> getLastAtomicElements() {
        if (getResolvedThis().isPresent() && getResolvedThis().get() != this) {
            return ((ArchitectureElementSymbol) getResolvedThis().get()).getLastAtomicElements();
        }
        else {
            return Collections.singletonList(this);
        }
    }

    @Override
    public Set<ParameterSymbol> resolve() throws ArchResolveException {
        if (!isResolved()) {
            if (isResolvable()) {
                getDeclaration();
                resolveExpressions();

                if (type == Type.IO) {
                    getIoDeclaration().getType().resolve();

                    if (!getArrayAccess().isPresent() && getIoDeclaration().getArrayLength() > 1) {
                        //transform io array into parallel composite
                        List<ArchitectureElementSymbol> parallelElements = createExpandedParallelElements();
                        ParallelCompositeElementSymbol composite = new ParallelCompositeElementSymbol();
                        composite.setElements(parallelElements);

                        getSpannedScope().getAsMutableScope().add(composite);
                        composite.setAstNode(getAstNode().get());

                        for (ArchitectureElementSymbol element : parallelElements) {
                            element.putInScope(composite.getSpannedScope());
                            element.setAstNode(getAstNode().get());
                        }

                        if (getInputElement().isPresent()) {
                            composite.setInputElement(getInputElement().get());
                        }

                        if (getOutputElement().isPresent()) {
                            composite.setOutputElement(getOutputElement().get());
                        }

                        composite.resolveOrError();

                        setResolvedThis(composite);
                    }
                    else {
                        //Add port to the ports stored in ArchitectureSymbol
                        if (getIoDeclaration().isInput()) {
                            getArchitecture().getInputs().add(this);
                        }
                        else {
                            getArchitecture().getOutputs().add(this);
                        }

                        setResolvedThis(this);
                    }
                }
                else if (type == Type.LAYER) {
                    // Sync input/output elements with LayerSymbol, if layer is called here (member == NONE) and LayerSymbol
                    // currently has no input/output elements. This is done so that we can use the first call of the layer
                    // to infer the shape of the out member and possibly other members in future.
                    if (getMember() == VariableSymbol.Member.NONE) {
                        if (!getLayerVariableDeclaration().getLayer().getInputElement().isPresent()
                                && getInputElement().isPresent()) {
                            getLayerVariableDeclaration().getLayer().setInputElement(getInputElement().get());
                        }

                        if (!getLayerVariableDeclaration().getLayer().getOutputElement().isPresent()
                                && getOutputElement().isPresent()) {
                            getLayerVariableDeclaration().getLayer().setOutputElement(getOutputElement().get());
                        }
                    }

                    // We skip the usage of LayerSymbol.computeOutputTypes() and LayerSymbol.checkInput() methods because
                    // we use these manually in VariableSymbol.computeOutputTypes() and VariableSymbol.checkInput().
                    // If we didn't do so, it'd be problematic when one LayerVariableDeclarationSymbol with its LayerSymbol
                    // is used in different instances of VariableSymbol (i.e. one call with member == NONE and one access
                    // to the layer's state via member == STATE)

                    getLayerVariableDeclaration().getLayer().resolveOrError();
                    LayerDeclarationSymbol layerDeclaration = getLayerVariableDeclaration().getLayer().getDeclaration();

                    if (!getArrayAccess().isPresent() && layerDeclaration.isPredefined() && ((PredefinedLayerDeclaration) layerDeclaration).getArrayLength(getMember()) > 1) {
                        List<ArchitectureElementSymbol> parallelElements = createExpandedParallelElements();
                        ParallelCompositeElementSymbol composite = new ParallelCompositeElementSymbol();
                        composite.setElements(parallelElements);

                        getSpannedScope().getAsMutableScope().add(composite);
                        composite.setAstNode(getAstNode().get());

                        for (ArchitectureElementSymbol element : parallelElements) {
                            element.putInScope(composite.getSpannedScope());
                            element.setAstNode(getAstNode().get());
                        }

                        if (getInputElement().isPresent()) {
                            composite.setInputElement(getInputElement().get());
                        }

                        if (getOutputElement().isPresent()) {
                            composite.setOutputElement(getOutputElement().get());
                        }

                        composite.resolveOrError();

                        setResolvedThis(composite);
                    }
                    else {
                        setResolvedThis(this);
                    }
                }
                else {
                    throw new ArchResolveException();
                }
            }
        }

        return getUnresolvableParameters();
    }

    private List<ArchitectureElementSymbol> createExpandedParallelElements() throws ArchResolveException {
        List<ArchitectureElementSymbol> parallelElements = new ArrayList<>();

        if (getType() == Type.IO) {
            IODeclarationSymbol ioDeclaration = (IODeclarationSymbol) getDeclaration();

            if (ioDeclaration.isInput()) {
                for (int i = 0; i < ioDeclaration.getArrayLength(); i++) {
                    VariableSymbol ioElement = new VariableSymbol(getName());
                    ioElement.setArrayAccess(i);
                    parallelElements.add(ioElement);
                }
            }
            else {
                for (int i = 0; i < ioDeclaration.getArrayLength(); i++){
                    SerialCompositeElementSymbol serialComposite = new SerialCompositeElementSymbol();

                    VariableSymbol ioElement = new VariableSymbol(getName());
                    ioElement.setArrayAccess(i);
                    ioElement.setAstNode(getAstNode().get());

                    LayerSymbol getLayer = new LayerSymbol(AllPredefinedLayers.GET_NAME);
                    getLayer.setArguments(Collections.singletonList(
                            new ArgumentSymbol.Builder()
                                    .parameter(AllPredefinedLayers.INDEX_NAME)
                                    .value(ArchSimpleExpressionSymbol.of(i))
                                    .build()));
                    getLayer.setAstNode(getAstNode().get());

                    serialComposite.setElements(Arrays.asList(getLayer, ioElement));

                    parallelElements.add(serialComposite);
                }
            }
        }
        else if (getType() == Type.LAYER) {
            LayerVariableDeclarationSymbol layerVariableDeclaration = getLayerVariableDeclaration();
            PredefinedLayerDeclaration predefinedLayerDeclaration =
                    (PredefinedLayerDeclaration) layerVariableDeclaration.getLayer().getDeclaration();
            int layerArrayLength = predefinedLayerDeclaration.getArrayLength(getMember());

            if (!getInputElement().isPresent()) {
                for (int i = 0; i < layerArrayLength; ++i) {
                    VariableSymbol element = new VariableSymbol(getName());
                    element.setArrayAccess(i);
                    element.setMember(getMember());
                    parallelElements.add(element);
                }
            }
            else {
                for (int i = 0; i < layerArrayLength; ++i) {
                    SerialCompositeElementSymbol serialComposite = new SerialCompositeElementSymbol();

                    VariableSymbol element = new VariableSymbol(getName());
                    element.setArrayAccess(i);
                    element.setMember(getMember());
                    element.setAstNode(getAstNode().get());

                    LayerSymbol getLayer = new LayerSymbol(AllPredefinedLayers.GET_NAME);
                    getLayer.setArguments(Collections.singletonList(
                            new ArgumentSymbol.Builder()
                                    .parameter(AllPredefinedLayers.INDEX_NAME)
                                    .value(ArchSimpleExpressionSymbol.of(i))
                                    .build()));
                    getLayer.setAstNode(getAstNode().get());

                    serialComposite.setElements(Arrays.asList(getLayer, element));

                    parallelElements.add(serialComposite);
                }
            }

        }

        return parallelElements;
    }

    @Override
    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters) {
        if (getArrayAccess().isPresent()) {
            getArrayAccess().get().checkIfResolvable(allParameters);
            unresolvableParameters.addAll(getArrayAccess().get().getUnresolvableParameters());
        }

        if (getType() == Type.IO) {
            ((IODeclarationSymbol) getDeclaration()).getType().checkIfResolvable(allParameters);
            unresolvableParameters.addAll(((IODeclarationSymbol) getDeclaration()).getType().getUnresolvableParameters());
        }
        else if (getType() == Type.LAYER) {
            ((LayerVariableDeclarationSymbol) getDeclaration()).getLayer().checkIfResolvable(allParameters);
            unresolvableParameters.addAll(((LayerVariableDeclarationSymbol) getDeclaration()).getLayer().getUnresolvableParameters());
        }
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes() {
        if (isAtomic()) {
            if (getType() == Type.IO) {
                // Allow inputs and outputs to be used as input
                if (isInput() || getOutputElement().isPresent()) {
                    return Collections.singletonList(((IODeclarationSymbol) getDeclaration()).getType());
                }
                else {
                    return Collections.emptyList();
                }
            }
            else if (getType() == Type.LAYER) {
                LayerSymbol layer = getLayerVariableDeclaration().getLayer();

                // if VariableSymbol has no output element and layer can be output, then return empty output types array
                if (!getOutputElement().isPresent()) {
                    if (((PredefinedLayerDeclaration) layer.getDeclaration()).canBeOutput(getMember())) {
                        return Collections.emptyList();
                    }
                }

                return ((PredefinedLayerDeclaration) layer.getDeclaration()).computeOutputTypes(getInputTypes(), layer, getMember());
            }
            else {
                throw new IllegalStateException("The architecture resolve() method was never called");
            }
        }
        else {
            if (!getResolvedThis().isPresent()) {
                throw new IllegalStateException("The architecture resolve() method was never called");
            }

            return ((ArchitectureElementSymbol) getResolvedThis().get()).computeOutputTypes();
        }
    }

    @Override
    public void checkInput() {
        if (isAtomic()) {
            if (getType() == Type.IO) {
                IODeclarationSymbol ioDeclaration = (IODeclarationSymbol) getDeclaration();

                if (isOutput()) {
                    String name = getName();
                    if (getArrayAccess().isPresent()) {
                        name = name + "[" + getArrayAccess().get().getIntValue().get() + "]";
                    }

                    if (getInputTypes().size() != 1) {
                        // Allow no input when output is used as input
                        if (!(getInputTypes().size() == 0 && getOutputElement().isPresent())) {
                            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid number of input streams. " +
                                            "The number of input streams for the output '" + name + "' is " + getInputTypes().size() + "."
                                    , getSourcePosition());
                        }
                    } else {
                        ASTElementType inputType = getInputTypes().get(0).getDomain();
                        ASTElementType outputType = ioDeclaration.getType().getDomain();
                        boolean error;
                        if(getInputElement().isPresent()){
                            error = !Utils.contains(outputType, inputType,getInputElement().get().getName());
                        }else{
                            error = !Utils.contains(outputType, inputType);
                        }

                        if (error) {
                            Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_DOMAIN + " " +
                                    "The declared output type of '" + name + "' does not match with the actual type. " +
                                    "Declared type: " + outputType.getName() + ". " +
                                    "Actual type: " + inputType.getName() + ".");
                        }
                    }
                }
            }
            else if (getType() == Type.LAYER) {
                LayerSymbol layer = getLayerVariableDeclaration().getLayer();

                if (!getInputElement().isPresent()) {
                    if (!((PredefinedLayerDeclaration) layer.getDeclaration()).canBeInput(getMember())) {
                        Log.error("0" + ErrorCodes.INVALID_ELEMENT_INPUT_SHAPE + " Invalid layer input. "
                                        + getMember().toString().toLowerCase() + " member needs an input",
                                getSourcePosition());
                    }
                }
                else {
                    ((PredefinedLayerDeclaration) layer.getDeclaration()).checkInput(getInputTypes(), layer, getMember());
                }
            }
        }
        else {
            if (!getResolvedThis().isPresent()) {
                throw new IllegalStateException("The architecture resolve() method was never called");
            }

            ((ArchitectureElementSymbol) getResolvedThis().get()).checkInput();
        }
    }

    @Override
    public Optional<Integer> getParallelLength() {
        if (getType() == Type.IO) {
            IODeclarationSymbol ioDeclaration = (IODeclarationSymbol) getDeclaration();
            return Optional.of(ioDeclaration.getArrayLength());
        }
        else if (getType() == Type.LAYER) {
            PredefinedLayerDeclaration predefinedLayerDeclaration =
                    (PredefinedLayerDeclaration) getLayerVariableDeclaration().getLayer().getDeclaration();
            return Optional.of(predefinedLayerDeclaration.getArrayLength(getMember()));
        }
        else {
            return Optional.empty();
        }
    }

    @Override
    public Optional<List<Integer>> getSerialLengths() {
        if (getType() != Type.UNKNOWN) {
            return Optional.of(Collections.nCopies(getParallelLength().get(), 1));
        }
        else {
            return Optional.empty();
        }
    }

    @Override
    protected void putInScope(Scope scope) {
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)) {
            scope.getAsMutableScope().add(this);
            if (getArrayAccess().isPresent()) {
                getArrayAccess().get().putInScope(getSpannedScope());
            }
        }
    }

    @Override
    protected void resolveExpressions() throws ArchResolveException {
        if (getArrayAccess().isPresent()){
            getArrayAccess().get().resolveOrError();

            if (!Constraints.INTEGER.check(getArrayAccess().get(), getSourcePosition(), getName()) ||
                !Constraints.NON_NEGATIVE.check(getArrayAccess().get(), getSourcePosition(), getName())) {
                throw new ArchResolveException();
            }
        }
    }

    @Override
    protected ArchitectureElementSymbol preResolveDeepCopy() {
        VariableSymbol copy = new VariableSymbol(getName());

        if (getAstNode().isPresent()) {
            copy.setAstNode(getAstNode().get());
        }

        copy.setMember(getMember());

        if (getArrayAccess().isPresent()) {
            copy.setArrayAccess(getArrayAccess().get().preResolveDeepCopy());
        }

        return copy;
    }
}
