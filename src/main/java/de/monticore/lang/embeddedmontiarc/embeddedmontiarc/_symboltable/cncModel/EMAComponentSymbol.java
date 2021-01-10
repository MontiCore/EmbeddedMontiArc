/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import com.google.common.collect.ImmutableList;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentKind;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.SymbolKind;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;
import static de.monticore.symboltable.Symbols.sortSymbolsByPosition;

/**
 * Symbol for component definitions.
 *
 */
public class EMAComponentSymbol extends CommonScopeSpanningSymbol implements EMAElementSymbol{

    public static final ComponentKind KIND = new ComponentKind();

    private final List<EMAComponentImplementationSymbol> implementations = new ArrayList<>();

    private boolean isInnerComponent = false;

    private Optional<EMAComponentSymbolReference> superComponent = Optional.empty();

    // when "this" not actually is a component, but a reference to a component, this optional
    // attribute is set by the symbol-table creator to the referenced component and must be used for
    // implementation.
    private Optional<EMAComponentSymbol> referencedComponent = Optional.empty();

    private List<ImportStatement> imports = new ArrayList<>();

    private List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols = new ArrayList<>();

    private List<EMAVariable> parameters = new ArrayList<>();

    private List<ASTExpression> arguments = new ArrayList<>();

    private List<ASTPortInitial> portInitials = new ArrayList<>();

    private List<ASTComponentModifier> componentModifiers = new ArrayList<>();

    public EMAComponentSymbol(String name) {
        super(name, KIND);
    }

    public EMAComponentSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    /**
     * @return referencedComponent
     */
    public Optional<EMAComponentSymbol> getReferencedComponent() {
        return this.referencedComponent;
    }

    /**
     * @param referencedComponent the referencedComponent to set
     */
    public void setReferencedComponent(Optional<EMAComponentSymbol> referencedComponent) {
        // to fix port instancing
        this.referencedComponent = referencedComponent;
    }

    /**
     * @param parameterType configuration parameter to add
     */
    public void addConfigParameter(MCFieldSymbol parameterType) {
        if (referencedComponent.isPresent())
            referencedComponent.get().addConfigParameter(parameterType);
        else {
            Log.errorIfNull(parameterType);
            checkArgument(parameterType.isParameter(), "Only parameters can be added.");
            getMutableSpannedScope().add(parameterType);
        }
    }

    /**
     * @param target target of the connector to get
     * @return a connector with the given target, absent optional, if it does not exist
     */
    public Optional<EMAConnectorSymbol> getConnector(String target) {
        // no check for reference required
        for (EMAConnectorSymbol con : getConnectors()) {
            if (con.getTarget().equals(target)) {
                return Optional.of(con);
            }
        }
        return Optional.empty();
    }

    /**
     * @return connectors of this component
     */
    public Collection<EMAConnectorSymbol> getConnectors() {
        Collection<EMAConnectorSymbol> c = referencedComponent.orElse(this)
                .getSpannedScope().<EMAConnectorSymbol>resolveLocally(EMAConnectorSymbol.KIND);

        return c.stream().sorted((o1, o2) -> o1.getSourcePosition().compareTo(o2.getSourcePosition()))
                .collect(Collectors.toList());
    }

    /**
     * @param visibility visibility
     * @return connectors with the given visibility
     */
    public Collection<EMAConnectorSymbol> getConnectors(AccessModifier visibility) {
        // no check for reference required
        return getConnectors().stream()
                .filter(c -> c.getAccessModifier().includes(visibility))
                .collect(Collectors.toList());
    }

    /**
     * Checks, if this component has a connector with the given receiver name.
     *
     * @param receiver name of the receiver to find a connector for
     * @return true, if this component has a connector with the given receiver name, else false.
     */
    public boolean hasConnector(String receiver) {
        // no check for reference required
        return getConnectors().stream()
                .filter(c -> c.getName().equals(receiver))
                .findAny().isPresent();
    }

    /**
     * Checks, if this component has one or more connectors with the given sender.
     *
     * @param sender name of the sender to find a connector for
     * @return true, if this component has one ore more connectors with the given sender name, else
     * false.
     */
    public boolean hasConnectors(String sender) {
        // no check for reference required
        return getConnectors().stream()
                .filter(c -> c.getSource().equals(sender))
                .findAny().isPresent();
    }

    /**
     * @param impl the implementation to add
     */
    public void addImplementation(EMAComponentImplementationSymbol impl) {
        referencedComponent.orElse(this).implementations.add(impl);
    }

    /**
     * @return implementations
     */
    public List<EMAComponentImplementationSymbol> getImplementations() {
        return ImmutableList.copyOf(referencedComponent.orElse(this).implementations);
    }

    public Optional<EMAComponentImplementationSymbol> getImplementation(String name) {
        // no check for reference required
        return getImplementations().stream()
                .filter(i -> i.getName().equals(name))
                .findFirst();
    }

    /**
     * @param visibility visibility
     * @return implementations with the given visibility
     */
    public Collection<EMAComponentImplementationSymbol> getImplementations(
            AccessModifier visibility) {
        // no check for reference required
        return getImplementations().stream()
                .filter(s -> s.getAccessModifier().includes(visibility))
                .collect(Collectors.toList());
    }

    /**
     * @return innerComponents
     */
    public Collection<EMAComponentSymbol> getInnerComponents() {
        return referencedComponent.orElse(this).getSpannedScope()
                .<EMAComponentSymbol>resolveLocally(EMAComponentSymbol.KIND);
    }

    /**
     * @param name inner component name
     * @return inner component with the given name, empty Optional, if it does not exist
     */
    public Optional<EMAComponentSymbol> getInnerComponent(String name) {
        // no check for reference required
        return getInnerComponents().stream()
                .filter(c -> c.getName().equals(name))
                .findFirst();
    }

    /**
     * @param visibility visibility
     * @return inner components with the given visibility
     */
    public Collection<EMAComponentSymbol> getInnerComponents(AccessModifier visibility) {
        // no check for reference require
        return getInnerComponents().stream()
                .filter(s -> s.getAccessModifier().includes(visibility))
                .collect(Collectors.toList());
    }

    /**
     * @return true, if this is an inner component, else false.
     */
    public boolean isInnerComponent() {
        return referencedComponent.orElse(this).isInnerComponent;
    }

    /**
     * Sets, if this is an inner component or not.
     *
     * @param isInnerComponent true, if this is an inner component
     */
    public void setIsInnerComponent(boolean isInnerComponent) {
        referencedComponent.orElse(this).isInnerComponent = isInnerComponent;
    }

    public void addFormalTypeParameter(MCTypeSymbol formalTypeParameter) {
        if (referencedComponent.isPresent()) {
            referencedComponent.get().addFormalTypeParameter(formalTypeParameter);
        } else {
            checkArgument(formalTypeParameter.isFormalTypeParameter());
            getMutableSpannedScope().add(formalTypeParameter);
        }
    }

    public List<MCTypeSymbol> getFormalTypeParameters() {
        final Collection<MCTypeSymbol> resolvedTypes = referencedComponent.orElse(this)
                .getSpannedScope().resolveLocally(MCTypeSymbol.KIND);
        return resolvedTypes.stream().filter(MCTypeSymbol::isFormalTypeParameter)
                .collect(Collectors.toList());
    }

    public boolean hasFormalTypeParameters() {
        return !getFormalTypeParameters().isEmpty();
    }

    public boolean hasConfigParameters() {
        return !getConfigParameters().isEmpty();
    }

    public boolean hasPorts() {
        return !getPortsList().isEmpty();
    }

    /**
     * Ports of this component.
     *
     * @return ports of this component.
     */
    public Collection<EMAPortSymbol> getPortsList() {
        Collection<EMAPortSymbol> symbols = referencedComponent.orElse(this).getSpannedScope()
                .<EMAPortSymbol>resolveLocally(EMAPortSymbol.KIND);
        /* for (EMAPortSymbol portSymbol : symbols) { System.out.println(portSymbol.toString()); } */
        return symbols;
    }

    /**
     * Ports of this component.
     *
     * @return ports of this component.
     */
    public Collection<EMAPortArraySymbol> getPortArrays() {
        Collection<EMAPortArraySymbol> symbols = referencedComponent.orElse(this).getSpannedScope()
                .<EMAPortArraySymbol>resolveLocally(EMAPortArraySymbol.KIND);
        return symbols;
    }

    public EMAPortArraySymbol getPortArray(String name) {
        Log.debug(name, "Looking for Pas:");
        for (EMAPortArraySymbol pas : getPortArrays()) {
            Log.debug(pas.getName(), "Cur Pas:");
            if (pas.getName().equals(name)) {
                Log.debug(pas.getName(), "Found Pas");
                return pas;
            }
        }
        return null;
    }

    public boolean isPortDependentOnResolutionDeclarationSymbol(String portName,
                                                                String nameToDependentOn) {
        EMAPortArraySymbol emaPortArraySymbol = getPortArray(portName);
        Log.debug(portName, "PortName:");
        Log.debug(nameToDependentOn, "Expected NameToDependOn:");
        if (emaPortArraySymbol.getNameSizeDependsOn().isPresent()) {
            Log.debug(emaPortArraySymbol.getNameSizeDependsOn().get(), "Actual NameToDependOn:");
            if (emaPortArraySymbol.getNameSizeDependsOn().get().equals(nameToDependentOn))
                return true;
        }
        return false;
    }

    // Overwrite in other classes
    public List<ActualTypeArgument> getActualTypeArguments() {
        return null;
    }
	
	/**
     * @param name port name
     * @return port with the given name, empty optional, if it does not exist
     */
    public Optional<EMAPortSymbol> getPort(String name) {
        // no check for reference required
        return getPortsList().stream()
                .filter(p -> p.getName().equals(name))
                .findFirst();
    }

    /**
     * @return incomingPorts of this component
     */
    public Collection<EMAPortSymbol> getIncomingPorts() {
        return referencedComponent.orElse(this).getSpannedScope()
                .<EMAPortSymbol>resolveLocally(EMAPortSymbol.KIND)
                .stream()
                .filter(p -> p.isIncoming())
                .collect(Collectors.toList());
    }

    /**
     * @param name port name
     * @return incoming port with the given name, empty optional, if it does not exist
     */
    public Optional<EMAPortSymbol> getIncomingPort(String name) {
        // no check for reference required
        return getIncomingPorts().stream()
                .filter(p -> p.getName().equals(name))
                .findFirst();
    }

    /**
     * @param visibility
     * @return incoming ports with the given visibility
     */
    public Collection<EMAPortSymbol> getIncomingPorts(AccessModifier visibility) {
        // no check for reference required
        return getIncomingPorts().stream()
                .filter(s -> s.getAccessModifier().includes(visibility))
                .collect(Collectors.toList());
    }

    /**
     * @return outgoingPorts of this component
     */
    public Collection<EMAPortSymbol> getOutgoingPorts() {
        return referencedComponent.orElse(this).getSpannedScope()
                .<EMAPortSymbol>resolveLocally(EMAPortSymbol.KIND)
                .stream()
                .filter(p -> p.isOutgoing())
                .collect(Collectors.toList());
    }

    /**
     * Returns a list of all incoming ports that also contains ports from a super component.
     *
     * @return list of all incoming ports.
     */
    public List<EMAPortSymbol> getAllIncomingPorts() {
        return referencedComponent.orElse(this).getAllPorts(true);
    }

    /**
     * @param name port name
     * @return outgoing port with the given name, empty optional, if it does not exist
     */
    public Optional<EMAPortSymbol> getOutgoingPort(String name) {
        // no check for reference required
        return getOutgoingPorts().stream()
                .filter(p -> p.getName().equals(name))
                .findFirst();
    }

    /**
     * @param visibility visibility
     * @return outgoing ports with the given visibility
     */
    public Collection<EMAPortSymbol> getOutgoingPorts(AccessModifier visibility) {
        // no check for reference required
        return getOutgoingPorts().stream()
                .filter(s -> s.getAccessModifier().includes(visibility))
                .collect(Collectors.toList());
    }

    /**
     * Returns a list of all outgoing ports that also contains ports from a super component.
     *
     * @return list of all outgoing ports.
     */
    public List<EMAPortSymbol> getAllOutgoingPorts() {
        return referencedComponent.orElse(this).getAllPorts(false);
    }

    protected List<EMAPortSymbol> getAllPorts() {
        List<EMAPortSymbol> result = new ArrayList<EMAPortSymbol>();

        // own ports
        result.addAll(getPortsList());

        // ports from super components
        Optional<EMAComponentSymbolReference> superCompOpt = getSuperComponent();
        if (superCompOpt.isPresent()) {
            for (EMAPortSymbol superPort : superCompOpt.get().getAllPorts()) {
                boolean alreadyAdded = false;
                for (EMAPortSymbol pToAdd : result) {
                    if (pToAdd.getName().equals(superPort.getName())) {
                        alreadyAdded = true;
                        break;
                    }
                }
                if (!alreadyAdded) {
                    result.add(superPort);
                }
            }
        }
        return result;
    }

    private List<EMAPortSymbol> getAllPorts(boolean isIncoming) {
        return getAllPorts().stream().filter(p -> p.isIncoming() == isIncoming)
                .collect(Collectors.toList());
    }

    /**
     * @return super component of this component, empty optional, if it does not have a super
     * component
     */
    public Optional<EMAComponentSymbolReference> getSuperComponent() {
        if (referencedComponent.isPresent()) {
            return referencedComponent.get().getSuperComponent();
        } else {
            return superComponent;
        }
    }

    /**
     * @param superComponent the super component to set
     */
    public void setSuperComponent(Optional<EMAComponentSymbolReference> superComponent) {
        referencedComponent.orElse(this).superComponent = superComponent;
    }

    /**
     * @return subComponents
     */
    public Collection<EMAComponentInstantiationSymbol> getSubComponents() {
        return referencedComponent.orElse(this).getSpannedScope()
                .resolveLocally(EMAComponentInstantiationSymbol.KIND);
    }

    /**
     * @param name subcomponent instance name
     * @return subcomponent with the given name, empty optional, if it does not exist
     */
    public Optional<EMAComponentInstantiationSymbol> getSubComponent(String name) {
        // no check for reference required
        return getSubComponents().stream()
                .filter(p -> p.getName().equals(name))
                .findFirst();
    }

    /**
     * @param visibility visibility
     * @return subcomponents with the given visibility
     */
    public Collection<EMAComponentInstantiationSymbol> getSubComponents(AccessModifier visibility) {
        // no check for reference required
        return getSubComponents().stream()
                .filter(s -> s.getAccessModifier().includes(visibility))
                .collect(Collectors.toList());
    }

    /**
     * @return configParameters
     */
    public List<MCFieldSymbol> getConfigParameters() {
        if (referencedComponent.isPresent()) {
            return referencedComponent.get().getConfigParameters();
        } else {
            final Collection<MCFieldSymbol> resolvedAttributes = getMutableSpannedScope()
                    .resolveLocally(MCFieldSymbol.KIND);
            final List<MCFieldSymbol> parameters = sortSymbolsByPosition(resolvedAttributes.stream()
                    .filter(MCFieldSymbol::isParameter).collect(Collectors.toList()));
            return parameters;
        }
    }

    /**
     * @return List of configuration parameters that are to be set during instantiation with the given
     * visibility
     */
    public Collection<MCFieldSymbol> getConfigParameters(AccessModifier visibility) {
        // no need to check for reference, as getParameres() does so.
        return getConfigParameters().stream()
                .filter(s -> s.getAccessModifier().includes(visibility))
                .collect(Collectors.toList());
    }

    public boolean isDecomposed() {
        return !isAtomic();
    }

    public boolean isAtomic() {
        return getSubComponents().isEmpty();
    }

    @Override
    public String toString() {
        return SymbolPrinter.printComponent(this);
    }

    /**
     * TODO reuse ArtifactScope? see TODO in {@link #setImports(List)}
     *
     * @return imports
     */
    public List<ImportStatement> getImports() {
        return this.imports;
    }

    /**
     * TODO could we get these somehow from the ArtifactScope? there the imports are private, but we
     * want (some?) imports to be printed in a generated java file, when e.g. aggregated with Java and
     * other Java-types are referenced.
     *
     * @param imports
     */
    public void setImports(List<ImportStatement> imports) {
        this.imports = imports;
    }

    public Optional<ResolutionDeclarationSymbol> getResolutionDeclarationSymbol(String name) {
        for (ResolutionDeclarationSymbol symbol : getResolutionDeclarationSymbols()) {
            if (symbol.getNameToResolve().equals(name))
                return Optional.of(symbol);
        }
        return Optional.empty();
    }

    public boolean hasResolutionDeclaration(String name) {
        for (ResolutionDeclarationSymbol resDeclSym : resolutionDeclarationSymbols)
            if (resDeclSym.getNameToResolve().equals(name)) {
                return true;
            }
        return false;
    }

    public int howManyResolutionDeclarationSymbol() {
        return resolutionDeclarationSymbols.size();
    }

    public void addResolutionDeclarationSymbol(
            ResolutionDeclarationSymbol resolutionDeclarationSymbol) {
        if (hasResolutionDeclaration(resolutionDeclarationSymbol.getNameToResolve())) {
            Log.error("0x0S0001 Name " + resolutionDeclarationSymbol.getNameToResolve()
                    + " to resolve is a duplicate");
        }
        resolutionDeclarationSymbols.add(resolutionDeclarationSymbol);
        Log.debug(getFullName(), "Added ResolutionDeclarationSymbol to EMAComponentSymbol with name:");
    }

    public List<ResolutionDeclarationSymbol> getResolutionDeclarationSymbols() {
        return resolutionDeclarationSymbols;
    }

    public static EMAComponentBuilder builder() {
        return EMAComponentBuilder.getInstance();
    }

    public void addParameter(ASTParameter astParameter) {

        if (referencedComponent.isPresent())
            referencedComponent.get().addParameter(astParameter);
        else {
            EMAVariable param = new EMAVariable();
            param.setName(astParameter.getNameWithArray().getName());
            param.setType(astParameter.getType());
            parameters.add(param);
        }
    }

    public List<EMAVariable> getParameters() {
        return parameters;
    }

    public void setParameters(List<EMAVariable> parameters) {
        this.parameters = parameters;
    }

    public List<ASTExpression> getArguments() {
        if (referencedComponent.isPresent())
            return referencedComponent.get().getArguments();
        return arguments;
    }

    public void addArgument(ASTExpression astExpression) {
        if (referencedComponent.isPresent())
            referencedComponent.get().addArgument(astExpression);
        else
            arguments.add(astExpression);
    }

    public void setArguments(List<ASTExpression> arguments) {
        this.arguments = arguments;
    }

    public void addIncomingPort(EMAPortSymbol symbol) {
        // TODO implement me
    }

    public Optional<EMAComponentSymbol> getParent() {
        return (Optional<EMAComponentSymbol>) getEnclosingScope().getSpanningSymbol();
    }

    public List<ASTPortInitial> getPortInitials() {
        if (referencedComponent.isPresent())
            return referencedComponent.get().getPortInitials();
        return portInitials;
    }

    public void addPortInitial(ASTPortInitial initialGuess) {
        if (referencedComponent.isPresent())
                referencedComponent.get().addPortInitial(initialGuess);
        else
            this.portInitials.add(initialGuess);
    }

    public void setPortInitials(List<ASTPortInitial> initalGuesses) {
        this.portInitials = initalGuesses;
    }


    public List<ASTComponentModifier> getComponentModifiers() {
        if (referencedComponent.isPresent())
            return referencedComponent.get().getComponentModifiers();
        return componentModifiers;
    }

    public void addComponentModifier(ASTComponentModifier componentModifier) {
        if (referencedComponent.isPresent())
            referencedComponent.get().addComponentModifier(componentModifier);
        else
            this.componentModifiers.add(componentModifier);
    }

    public void setComponentModifiers(List<ASTComponentModifier> componentModifiers) {
        this.componentModifiers = componentModifiers;
    }

    public boolean isNonVirtual() {
        for (ASTComponentModifier componentModifier : componentModifiers) {
            if (componentModifier instanceof ASTVirtModifier) {
                if (((ASTVirtModifier) componentModifier).getVIRTUAL().equals(ASTVIRTUAL.NONVIRTUAL))
                    return true;
                else
                    return false;
            }
        }
        return false;
    }

    public boolean isVirtual() {
        return !isNonVirtual();
    }

    public boolean isNonDirectFeedThrough() {
        for (ASTComponentModifier componentModifier : componentModifiers) {
            if (componentModifier instanceof ASTDFModifier) {
                if (((ASTDFModifier) componentModifier).getDIRECTFEEDTHROUGH().equals(ASTDIRECTFEEDTHROUGH.NONDF))
                    return true;
                else
                    return false;
            }
        }
        return false;
    }

    public boolean isDirectFeedThrough() {
        return !isNonDirectFeedThrough();
    }
}
