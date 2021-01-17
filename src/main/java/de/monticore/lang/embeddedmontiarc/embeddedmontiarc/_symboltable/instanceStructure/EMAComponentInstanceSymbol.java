/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.se_rwth.commons.Joiners;

import java.util.*;
import java.util.stream.Collectors;

/**
 *         Created by Michael von Wenckstern on 23.05.2016.
 *         The aim of this class is to have real component instances<br>
 *         component A {
 *         component B<Integer> b1;
 *         component B<Double> b2;
 *         }
 *         component B<T> {
 *         component C<T> c1, c2;
 *         }
 *         component C<T> {
 *         ports in T a,
 *         in T b,
 *         out T c;
 *         }
 *         after expanding the component's definitions we will get the following instance:
 *         component instance A {
 *         component instance b1 {
 *         component instance c1 {
 *         ports in Integer a,
 *         ports in Integer b,
 *         ports out Integer c;
 *         }
 *         component instance c2 {
 *         ports in Integer a,
 *         ports in Integer b,
 *         ports out Integer c;
 *         }
 *         }
 *         component instance b2 {
 *         component instance c1 {
 *         ports in Double a,
 *         ports in Double b,
 *         ports out Double c;
 *         }
 *         component instance c2 {
 *         ports in Double a,
 *         ports in Double b,
 *         ports out Double c;
 *         }
 *         }
 *         }
 *         These instances are important for:
 *         * executing the simulation order later on (to be compatible with industry, Simulink:
 *         they only care about the atomic elements and give them an execution order.
 *         In this example these would be the four elements: A.b1.c1, A.b1.c2, A.b2.c1, A.b2.c2
 *         *  for tagging instances differently (e.g. A.b1.c1 may be deployed on another processor
 *         than A.b2.c1)
 *         *  different C%C analysis techniques, e.g. Control-Flow-Analysis (need to differentiate
 *         between instances)
 *         This class is the basic class for instances so that you can resolve them using the
 *         standard symbol table mechanism
 */
public class EMAComponentInstanceSymbol
        extends CommonScopeSpanningSymbol implements EMAElementInstanceSymbol {

    public static final EMAComponentInstanceKind KIND = new EMAComponentInstanceKind();

    protected EMAComponentSymbolReference type;
    protected List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();
    protected List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols;
    protected List<EMAVariable> parameters = new ArrayList<>();
    protected List<ASTExpression> arguments = new ArrayList<>();

    protected List<ASTComponentModifier> componentModifiers = new ArrayList<>();

    protected List<Integer> orderOutput = new ArrayList<>();
    protected Integer orderUpdate = Integer.MAX_VALUE;

    /**
     * use {@link #builder()}
     */
    protected EMAComponentInstanceSymbol(String name, EMAComponentSymbolReference type) {
        super(name, KIND);
        this.type = type;
    }

    public static EMAComponentInstanceBuilder builder() {
        return new EMAComponentInstanceBuilder();
    }

    /**
     * this is only needed as temp variable to derive generics
     */
    @Deprecated
    public List<ActualTypeArgument> getActualTypeArguments() {
        return actualTypeArguments;
    }

    public void setActualTypeArguments(List<ActualTypeArgument> actualTypeArguments) {
        this.actualTypeArguments = actualTypeArguments;
    }

    public EMAComponentSymbolReference getComponentType() {
        return type;
    }

    public boolean hasPorts() {
        return !getPortInstanceList().isEmpty();
    }

    public List<ResolutionDeclarationSymbol> getResolutionDeclarationSymbols() {
        return resolutionDeclarationSymbols;
    }

    public ResolutionDeclarationSymbol getResolutionDeclarationSymbol(String name) {
        for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : resolutionDeclarationSymbols) {
            if (resolutionDeclarationSymbol.getNameToResolve().equals(name))
                return resolutionDeclarationSymbol;
        }

        return null;
    }

    public Optional<InstanceInformation> getInstanceInformation() {
        return InstancingRegister.getInstanceInformation(getName());
    }


    public Collection<EMAPortInstanceSymbol> getPortInstanceList() {
        return getSpannedScope().<EMAPortInstanceSymbol>resolveLocally(EMAPortInstanceSymbol.KIND);
    }

    public Collection<EMAPortArraySymbol> getPortArrays() {
        return getSpannedScope().<EMAPortArraySymbol>resolveLocally(EMAPortArraySymbol.KIND);
    }

    public Optional<EMAPortInstanceSymbol> getPortInstance(String name) {
        return getSpannedScope().resolveLocally(name, EMAPortInstanceSymbol.KIND);
    }

    public Collection<EMAPortInstanceSymbol> getIncomingPortInstances() {
        return getPortInstanceList().stream().filter(EMAPortInstanceSymbol::isIncoming).collect(Collectors.toList());
    }

    public Optional<EMAPortInstanceSymbol> getIncomingPortInstance(String name) {
        return getIncomingPortInstances().stream().filter(p -> p.getName().equals(name)).findFirst();
    }

    public Collection<EMAPortInstanceSymbol> getOutgoingPortInstances() {
        return getPortInstanceList().stream().filter(EMAPortInstanceSymbol::isOutgoing).collect(Collectors.toList());
    }

    public Optional<EMAPortInstanceSymbol> getOutgoingPortInstance(String name) {
        return getOutgoingPortInstances().stream().filter(p -> p.getName().equals(name)).findFirst();
    }


    public Collection<EMAComponentInstanceSymbol> getSubComponents() {
        return getSpannedScope().<EMAComponentInstanceSymbol>resolveLocally(EMAComponentInstanceSymbol.KIND);
    }

    public Optional<EMAComponentInstanceSymbol> getSubComponent(String name) {
        return getSpannedScope().<EMAComponentInstanceSymbol>resolveLocally(name, EMAComponentInstanceSymbol.KIND);
    }

    /**
     * EMAComponentInstanceSymbol::getPortInstanceList() may return different
     * results than EMAComponentSymbol::getPortInstanceList()
     * "MontiArc provides a structural inheritance mechanism that allows to define a component as
     * an extension of another component type (see requirement LRQ1.1.1). The new type inherits the
     * interface as well as the architectural configuration from the supercomponent. Thus, all ports,
     * inner component type definitions, subcomponents, and connectors are inherited." (p. 42, Ph.D. AH)
     */
    public Collection<EMAConnectorInstanceSymbol> getConnectorInstances() {
        return getSpannedScope().<EMAConnectorInstanceSymbol>resolveLocally(EMAConnectorInstanceSymbol.KIND);
    }

    @Override
    public String toString() {
        return SymbolPrinter.printEMAComponentInstance(this);
    }


    public boolean containsPort(EMAPortInstanceSymbol emaPortSymbol) {
        for (EMAPortInstanceSymbol symbol : getPortInstanceList())
            if (symbol.equals(emaPortSymbol))
                return true;
        return false;
    }

    /**
     * returns the connectors which connect an in or output port of a subcomponent of this component
     * to another subcomponent's port
     *
     * @return
     */
    public List<EMAConnectorInstanceSymbol> getSubComponentConnectors() {
        Set<EMAConnectorInstanceSymbol> set = new LinkedHashSet<>();

        Collection<EMAConnectorInstanceSymbol> connectors = getConnectorInstances();
        Collection<EMAComponentInstanceSymbol> subComponents = getSubComponents();

        for (EMAConnectorInstanceSymbol connector : connectors) {
            EMAPortInstanceSymbol sourcePort = connector.getSourcePort();
            EMAPortInstanceSymbol targetPort = connector.getTargetPort();
            EMAComponentInstanceSymbol sourceCmp = sourcePort.getComponentInstance();
            EMAComponentInstanceSymbol targetCmp = targetPort.getComponentInstance();
            if (subComponents.contains(sourceCmp) && subComponents.contains(targetCmp)) {
                set.add(connector);
            }

        }

        return new ArrayList<>(set);
    }


    public List<EMAComponentInstanceSymbol> getIndependentSubComponents() {
        Collection<EMAComponentInstanceSymbol> subComponents = getSubComponents();
        List<EMAConnectorInstanceSymbol> subComponentConnectors = getSubComponentConnectors();

        Set<EMAComponentInstanceSymbol> nonIndependentSubComponents = new HashSet<>();
        for (EMAConnectorInstanceSymbol connector : subComponentConnectors) {
            EMAPortInstanceSymbol sourcePort = connector.getSourcePort();
            EMAComponentInstanceSymbol sourceCmp = sourcePort.getComponentInstance();
            nonIndependentSubComponents.add(sourceCmp);

        }

        List<EMAComponentInstanceSymbol> independentSubComponents = new ArrayList<>(subComponents);
        independentSubComponents.removeAll(nonIndependentSubComponents);
        return new ArrayList<>(independentSubComponents);
    }


    public int getUnitNumberResolutionSubComponents(String name) {
        ASTUnitNumberResolution unitNumberResolution = (ASTUnitNumberResolution) getSubComponents().iterator().next().getComponentType().getReferencedSymbol().getResolutionDeclarationSymbol(name).get().getASTResolution();

        return unitNumberResolution.getNumber().get().intValue();
    }

    public List<ResolutionDeclarationSymbol> getResolutionDeclarationsSubComponent(String name) {
        return getSubComponent(name).get().getComponentType().getReferencedSymbol().getResolutionDeclarationSymbols();
    }

    public void setResolutionDeclarationSymbols(List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols) {
        this.resolutionDeclarationSymbols = resolutionDeclarationSymbols;
    }

    public boolean isSubComponent(String name) {
        for (EMAComponentInstanceSymbol subComponent : getSubComponents()) {
            if (subComponent.getFullName().equals(name))
                return true;
        }
        return false;
    }


    public List<EMAVariable> getParameters() {
        return parameters;
    }

    public void setParameters(List<EMAVariable> parameters) {
        this.parameters = parameters;
    }

    public List<ASTExpression> getArguments() {
        return arguments;
    }

    public void setArguments(List<ASTExpression> arguments) {
        this.arguments = arguments;
    }

    public List<ASTComponentModifier> getComponentModifiers() {
        return componentModifiers;
    }

    public void setComponentModifiers(List<ASTComponentModifier> componentModifiers) {
        this.componentModifiers = componentModifiers;
    }

    public List<Integer> getOrderOutput() {
        return orderOutput;
    }

    public void setOrderOutput(List<Integer> orderOutput) {
        this.orderOutput = orderOutput;
    }

    public void addOrderOutput(Integer orderOutput) {
        if (!this.orderOutput.contains(orderOutput))
            this.orderOutput.add(orderOutput);
    }

    public Integer getOrderUpdate() {
        return orderUpdate;
    }

    public void setOrderUpdate(Integer orderUpdate) {
        this.orderUpdate = orderUpdate;
    }

    public Optional<EMAComponentInstanceSymbol> getEnclosingComponent() {
        if (getEnclosingScope() == null) return Optional.empty();
        return (Optional<EMAComponentInstanceSymbol>) getEnclosingScope().getSpanningSymbol();
    }

    public Optional<EMAComponentInstanceSymbol> getParent() {
        if (getEnclosingScope() == null) return Optional.empty();
        return (Optional<EMAComponentInstanceSymbol>) getEnclosingScope().getSpanningSymbol();
    }


    public boolean isNonVirtual() {
        if (!getParent().isPresent()) return true;
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

    public void resetFullNames() {
        setFullName(Joiners.DOT.join(this.getPackageName(), this.getName()));
        for (EMAComponentInstanceSymbol subComponent : getSubComponents()) {
            subComponent.setPackageName(getFullName());
            subComponent.resetFullNames();
        }
    }
}
