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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

/**
 * @author Michael von Wenckstern
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

    protected ComponentSymbolReference type;
    protected List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();
    protected List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols;
    protected List<EMAVariable> parameters = new ArrayList<>();
    protected List<ASTExpression> arguments = new ArrayList<>();

    /**
     * use {@link #builder()}
     */
    protected EMAComponentInstanceSymbol(String name, ComponentSymbolReference type) {
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

    public ComponentSymbolReference getComponentType() {
        return type;
    }

    public boolean hasPorts() {
        return !getPortsList().isEmpty();
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

    public void fixWrongPortsInInstances() {
        for (EMAComponentInstanceSymbol instanceSymbol : getSubComponents()) {
            instanceSymbol.fixWrongPortsInInstances();

        }
        MutableScope scope = getSpannedScope().getAsMutableScope();
        Log.debug(toString(), "Current Instance");
        //TODO fix to work for more than one arguments

        for (EMAPortSymbol emaPortSymbolMain : getPortsList()) {
            int counter = 1;
            InstanceInformation info = InstancingRegister.getInstanceInformation(getName()).orElse(null);
            int number = -1;
            if (info != null) {
                Log.debug(info.getInstanceNumberForArgumentIndex(0) + "", "Instance Information");
                //number = info.getInstanceNumberForArgumentIndex(0);
                number = info.getInstanceNumberForPortName(emaPortSymbolMain.getNameWithoutArrayBracketPart());
            } else {
                Log.info("No instance information for " + emaPortSymbolMain.getName(), "Missing:");
            }
            for (EMAPortSymbol emaPortSymbol : getPortsList()) {
                if (emaPortSymbol.getName().startsWith(emaPortSymbolMain.getNameWithoutArrayBracketPart() + "[") && emaPortSymbol.isPartOfPortArray()) {
                    if (number > -1 && counter > number) {
                        scope.remove(emaPortSymbol);
                        Log.info(emaPortSymbol.getName(), "Removed:");
                    }
                    ++counter;
                }
            }
            for (int i = 1; i <= number; ++i) {
                if (!getPort(emaPortSymbolMain.getNameWithoutArrayBracketPart() + "[" + i + "]").isPresent()) {
                    EMAPortSymbol emaPortSymbolNew = new EMAPortSymbol(emaPortSymbolMain.getNameWithoutArrayBracketPart() + "[" + i + "]");
                    emaPortSymbolNew.setTypeReference(emaPortSymbolMain.getTypeReference());
                    emaPortSymbolNew.setNameDependsOn(emaPortSymbolMain.getNameDependsOn());
                    emaPortSymbolNew.setDirection(emaPortSymbolMain.isIncoming());

                    scope.add(emaPortSymbolNew);
                }
            }
        }
    }

    /**
     * EMAComponentInstanceSymbol::getPortsList() may return different
     * results than ComponentSymbol::getPortsList()
     * "MontiArc provides a structural inheritance mechanism that allows to define a component as
     * an extension of another component type (see requirement LRQ1.1.1). The new type inherits the
     * interface as well as the architectural configuration from the supercomponent. Thus, all ports,
     * inner component type definitions, subcomponents, and connectors are inherited." (p. 42, Ph.D. AH)
     */
    public Collection<EMAPortSymbol> getPortsList() {
        return getSpannedScope().<EMAPortSymbol>resolveLocally(EMAPortSymbol.KIND);
    }

    public Collection<PortArraySymbol> getPortArrays() {
        return getSpannedScope().<PortArraySymbol>resolveLocally(PortArraySymbol.KIND);
    }

    public Optional<EMAPortSymbol> getPort(String name) {
        return getSpannedScope().resolveLocally(name, EMAPortSymbol.KIND);
    }

    public Collection<EMAPortSymbol> getIncomingPorts() {
        return getPortsList().stream().filter(EMAPortSymbol::isIncoming).collect(Collectors.toList());
    }

    public Optional<EMAPortSymbol> getIncomingPort(String name) {
        // no check for reference required
        return getIncomingPorts().stream().filter(p -> p.getName().equals(name)).findFirst();
    }

    public Collection<EMAPortSymbol> getOutgoingPorts() {
        return getPortsList().stream().filter(EMAPortSymbol::isOutgoing).collect(Collectors.toList());
    }

    public Optional<EMAPortSymbol> getOutgoingPort(String name) {
        // no check for reference required
        return getOutgoingPorts().stream().filter(p -> p.getName().equals(name)).findFirst();
    }

    /**
     * EMAComponentInstanceSymbol::getSubComponents() may return different
     * results than the union of ComponentSymbol::getSubComponents() and
     * ComponentSymbol::getInnerComponents.
     * "MontiArc provides a structural inheritance mechanism that allows to define a component as
     * an extension of another component type (see requirement LRQ1.1.1). The new type inherits the
     * interface as well as the architectural configuration from the supercomponent. Thus, all ports,
     * inner component type definitions, subcomponents, and connectors are inherited." (p. 42, Ph.D. AH)
     */
    public Collection<EMAComponentInstanceSymbol> getSubComponents() {
        return getSpannedScope().<EMAComponentInstanceSymbol>resolveLocally(EMAComponentInstanceSymbol.KIND);
    }

    public Optional<EMAComponentInstanceSymbol> getSubComponent(String name) {
        return getSpannedScope().<EMAComponentInstanceSymbol>resolveLocally(name, EMAComponentInstanceSymbol.KIND);
    }

    /**
     * EMAComponentInstanceSymbol::getPortsList() may return different
     * results than ComponentSymbol::getPortsList()
     * "MontiArc provides a structural inheritance mechanism that allows to define a component as
     * an extension of another component type (see requirement LRQ1.1.1). The new type inherits the
     * interface as well as the architectural configuration from the supercomponent. Thus, all ports,
     * inner component type definitions, subcomponents, and connectors are inherited." (p. 42, Ph.D. AH)
     */
    public Collection<ConnectorSymbol> getConnectors() {
        return getSpannedScope().<ConnectorSymbol>resolveLocally(ConnectorSymbol.KIND);
    }

    @Override
    public String toString() {
        return SymbolPrinter.printEMAComponentInstance(this);
    }
    /*
    @Override
    public void addTag(TagSymbol tag) {
        Map localSymbols = this.getMutableSpannedScope().getLocalSymbols();
        if(localSymbols.get(tag.getName()) == null || !((Collection)localSymbols.get(tag.getName())).contains(tag)) {
            Log.info(this.getMutableSpannedScope().toString(),"Scope Before Add :");
            this.getMutableSpannedScope().add(tag);
            Log.info(this.getMutableSpannedScope().toString(),"Scope After Add :");
            Log.info("size: "+getTags((TagKind) tag.getKind()).size()+" "+getTags().contains(tag)+"","Contains Added Tag:");
        }else{
            Log.info(tag.getName(),"Tag was not added:");
        }

    }*/

    public boolean containsPort(EMAPortSymbol emaPortSymbol) {
        for (EMAPortSymbol symbol : getPortsList())
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
    public List<ConnectorSymbol> getSubComponentConnectors() {
        Set<ConnectorSymbol> set = new LinkedHashSet<>();

        Collection<ConnectorSymbol> connectors = getConnectors();
        Collection<EMAComponentInstanceSymbol> subComponents = getSubComponents();

        for (ConnectorSymbol connector : connectors) {
            EMAPortSymbol sourcePort = connector.getSourcePort();
            EMAPortSymbol targetPort = connector.getTargetPort();
            Optional<EMAComponentInstanceSymbol> sourceCmpOpt = sourcePort.getComponentInstance();
            Optional<EMAComponentInstanceSymbol> targetCmpOpt = targetPort.getComponentInstance();

            if (sourceCmpOpt.isPresent() && targetCmpOpt.isPresent()) {
                EMAComponentInstanceSymbol sourceCmp = sourceCmpOpt.get();
                EMAComponentInstanceSymbol targetCmp = targetCmpOpt.get();
                if (subComponents.contains(sourceCmp) && subComponents.contains(targetCmp)) {
                    set.add(connector);
                }
            }
        }

        return new ArrayList<>(set);
    }


    public List<EMAComponentInstanceSymbol> getIndependentSubComponents() {
        Collection<EMAComponentInstanceSymbol> subComponents = getSubComponents();
        List<ConnectorSymbol> subComponentConnectors = getSubComponentConnectors();

        Set<EMAComponentInstanceSymbol> nonIndependentSubComponents = new HashSet<>();
        for (ConnectorSymbol connector : subComponentConnectors) {
            EMAPortSymbol sourcePort = connector.getSourcePort();
            Optional<EMAComponentInstanceSymbol> sourceCmpOpt = sourcePort.getComponentInstance();
            if (sourceCmpOpt.isPresent()) {
                EMAComponentInstanceSymbol sourceCmp = sourceCmpOpt.get();
                nonIndependentSubComponents.add(sourceCmp);
            }
        }

        List<EMAComponentInstanceSymbol> independentSubComponents = new ArrayList<>(subComponents);
        independentSubComponents.removeAll(nonIndependentSubComponents);
        return new ArrayList<>(independentSubComponents);
    }

    public boolean isTemplateComponent() {
        return getActualTypeArguments().size() > 0;
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

    public Optional<EMAComponentInstanceSymbol> getEnclosingComponent() {
        return (Optional<EMAComponentInstanceSymbol>) getEnclosingScope().getSpanningSymbol();
    }
}
