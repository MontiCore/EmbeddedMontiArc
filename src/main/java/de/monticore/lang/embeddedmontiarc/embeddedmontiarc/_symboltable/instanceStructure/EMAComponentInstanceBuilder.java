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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Created by Michael von Wenckstern on 23.05.2016.
 */
public class EMAComponentInstanceBuilder {
    protected Optional<String> name = Optional.empty();
    protected Optional<EMAComponentSymbolReference> symbolReference = Optional.empty();
    protected List<EMAPortSymbol> ports = new ArrayList<>();
    protected List<EMAComponentInstanceSymbol> subComponents = new ArrayList<>();
    protected List<EMAConnectorSymbol> connectors = new ArrayList<>();
    protected Set<ResolvingFilter> resolvingFilters = new LinkedHashSet<>();
    //             FormalTypeParameter, ActualTypeArgument (is the binding of formal parameters
    protected Map<MCTypeSymbol, ActualTypeArgument> actualTypeArguments = new LinkedHashMap<>();
    protected List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols = new ArrayList<>();
    protected List<EMAVariable> parameters = new ArrayList<>();
    protected List<ASTExpression> arguments = new ArrayList<>();
    protected String packageName = "";

    protected static Map<MCTypeSymbol, ActualTypeArgument> createMap(List<MCTypeSymbol> keys, List<ActualTypeArgument> values) {
        Map<MCTypeSymbol, ActualTypeArgument> ret = new LinkedHashMap<>();
        for (int i = 0; i < keys.size(); i++) {
            ret.put(keys.get(i), values.get(i));
        }
        return ret;
    }

    public static EMAComponentInstanceSymbol clone(EMAComponentInstanceSymbol inst) {
        return new EMAComponentInstanceBuilder().setName(inst.getName())
                .setSymbolReference(inst.getComponentType())
                //.addPorts(inst.getPortInstanceList().stream().map(p -> EMAPortBuilder.clone(p)).collect(Collectors.toList()))
                //.addPorts(inst.getPortInstanceList()) // is cloned in build method
                .addConnectors(inst.getConnectorInstances().stream().map(c -> EMAConnectorBuilder.clone(c)).collect(Collectors.toList()))
                .addSubComponents(inst.getSubComponents().stream().map(s -> EMAComponentInstanceBuilder.clone(s)).collect(Collectors.toList()))
                .addResolutionDeclarationSymbols(inst.getResolutionDeclarationSymbols())
                .build();
    }

    public EMAComponentInstanceBuilder addResolvingFilter(ResolvingFilter filter) {
        this.resolvingFilters.add(filter);
        return this;
    }

    public EMAComponentInstanceBuilder addResolvingFilters(Set<ResolvingFilter<? extends Symbol>> filters) {
        for (ResolvingFilter filter : filters) {
            this.addResolvingFilter(filter);
        }
        return this;
    }

    public EMAComponentInstanceBuilder setName(String name) {
        this.name = Optional.of(name);
        return this;
    }

    public EMAComponentInstanceBuilder setSymbolReference(EMAComponentSymbolReference symbolReference) {
        this.symbolReference = Optional.of(symbolReference);
        return this;
    }

    public EMAComponentInstanceBuilder addPort(EMAPortSymbol port) {
        this.ports.add(port);
        return this;
    }

    public EMAComponentInstanceBuilder addPorts(EMAPortSymbol... ports) {
        for (EMAPortSymbol p : ports) {
            this.addPort(p);
        }
        return this;
    }

    public EMAComponentInstanceBuilder addPorts(Collection<EMAPortSymbol> ports) {
        ports.stream().forEachOrdered(p -> this.addPort(p));
        return this;
    }

    public EMAComponentInstanceBuilder addActualTypeArgument(MCTypeSymbol formalTypeParameter, ActualTypeArgument typeArgument) {
        this.actualTypeArguments.put(formalTypeParameter, typeArgument);
        return this;
    }

    public EMAComponentInstanceBuilder addActualTypeArguments(List<MCTypeSymbol> formalTypeParameters, List<ActualTypeArgument> actualTypeArguments) {
        if (formalTypeParameters.size() != actualTypeArguments.size()) {
            Log.debug(formalTypeParameters.toString(), "FormalTypeParameters");
            Log.debug(actualTypeArguments.toString(), "ActualTypeArguments");
            Log.debug("instance has not as many actual type arguments as component definition has formal type parameters. No mapping is possible. Function does nothing.",
                    EMAComponentInstanceBuilder.class.toString());
        } else {
            for (int i = 0; i < formalTypeParameters.size(); i++) {
                this.addActualTypeArgument(formalTypeParameters.get(i), actualTypeArguments.get(i));
            }
        }
        return this;
    }

    public EMAComponentInstanceBuilder addPortsIfNameDoesNotExists(Collection<EMAPortSymbol> ports) {
        List<String> existingPortNames = this.ports.stream().map(p -> p.getName())
                .collect(Collectors.toList());
        this.addPorts(ports.stream().filter(p ->
                !existingPortNames.contains(p.getName()))
                .collect(Collectors.toList()));
        return this;
    }

    /**
     * adds ports if they do not exist and replace generics of ports
     */
    public EMAComponentInstanceBuilder addPortsIfNameDoesNotExists(Collection<EMAPortSymbol> ports, List<MCTypeSymbol> formalTypeParameters, List<ActualTypeArgument> actualTypeArguments) {
        List<EMAPortSymbol> pList = ports.stream().collect(Collectors.toList());
        createMap(formalTypeParameters, actualTypeArguments).forEach((k, v) ->
                ports.stream().filter(p -> p.getTypeReference().getReferencedSymbol().getName().equals(k.getName()))
                        .forEachOrdered(p -> {
                            EMAPortSymbol pCloned = EMAPortBuilder.clone(p);
                            pCloned.setTypeReference((MCTypeReference<? extends MCTypeSymbol>) v.getType());
                            Collections.replaceAll(pList, p, pCloned);
                        })
        );
        this.addPortsIfNameDoesNotExists(pList);
        return this;
    }

    public EMAComponentInstanceBuilder addSubComponent(EMAComponentInstanceSymbol subCmp) {
        this.subComponents.add(subCmp);
        return this;
    }

    public EMAComponentInstanceBuilder addSubComponentIfNameDoesNotExists(EMAComponentInstanceSymbol subCmp) {
        List<String> existingSubComponentNames = this.subComponents.stream().map(s -> s.getName())
                .collect(Collectors.toList());
        if (!existingSubComponentNames.contains(subCmp.getName())) {
            this.addSubComponent(subCmp);
        }
        return this;
    }

    public EMAComponentInstanceBuilder addSubComponents(EMAComponentInstanceSymbol... subCmps) {
        for (EMAComponentInstanceSymbol s : subCmps) {
            this.addSubComponent(s);
        }
        return this;
    }

    public EMAComponentInstanceBuilder addSubComponents(Collection<EMAComponentInstanceSymbol> subCmps) {
        subCmps.stream().forEachOrdered(s -> this.addSubComponent(s));
        return this;
    }

    public EMAComponentInstanceBuilder addSubComponentsIfNameDoesNotExists(Collection<EMAComponentInstanceSymbol> subCmps) {
        List<String> existingSubComponentNames = this.subComponents.stream().map(s -> s.getName())
                .collect(Collectors.toList());
        this.addSubComponents(subCmps.stream().filter(s ->
                !existingSubComponentNames.contains(s.getName()))
                .collect(Collectors.toList()));
        return this;
    }

    public EMAComponentInstanceBuilder addConnector(EMAConnectorSymbol connector) {
        this.connectors.add(connector);
        return this;
    }

    public EMAComponentInstanceBuilder addConnectors(EMAConnectorSymbol... connectors) {
        for (EMAConnectorSymbol c : connectors) {
            this.addConnector(c);
        }
        return this;
    }

    public EMAComponentInstanceBuilder addConnectors(Collection<EMAConnectorSymbol> connectors) {
        connectors.stream().forEachOrdered(c -> this.addConnector(c));
        return this;
    }

    protected void exchangeGenerics(EMAComponentInstanceSymbol inst,
                                    Map<MCTypeSymbol, ActualTypeArgument> mapTypeArguments) {
        Log.debug(inst.toString(), "exchangeGenerics inst");
        // TODO work with full names, but then you got the problem with generics.GenericInstance.Generic.T != generics.SuperGenericComparableComp2.T
        // because when delegating the name of the referenced type must be created

        mapTypeArguments.forEach((k, v) -> {
            // 1) replace port generics
            inst.getPortInstanceList().stream()
                    //          .filter(p -> p.getTypeReference().getReferencedSymbol().getFullName().equals(k.getFullName()))
                    .filter( p -> p.getTypeReference().existsReferencedSymbol() ? p.getTypeReference().getReferencedSymbol().getName().equals(k.getName()) : false)
                    .forEachOrdered(p -> p.setTypeReference((MCTypeReference<? extends MCTypeSymbol>) v.getType()));

            // 2) propagate component instance definition generics
            inst.getSubComponents().stream()
                    // now update the actual type reference definitions by replacing them according to the hash map
                    .forEachOrdered(s -> s.setActualTypeArguments(
                            s.getActualTypeArguments().stream()
                                    // replace this filtered type arguments with the value we want to replace
                                    //                  .map(a -> a.getType().getReferencedSymbol().getFullName().equals(k.getFullName()) ? v : a)
                                    .map(a -> (a.getType().existsReferencedSymbol() ? (a.getType().getReferencedSymbol().getName().equals(k.getName()) ? v : a) : a))
                                    .collect(Collectors.toList())
                    ));

        });

        // delegate generic exchanges through inner component hierarchy
        inst.getSubComponents().stream()
                .forEachOrdered(s -> {
                    if (s.getActualTypeArguments().size() != s.getComponentType().getFormalTypeParameters().size()) {
                        /*Log.error(String.format("instance '%s' has a subcomponent instance '%s' where the given generics '%s' distinguish from the generics definition '%s'",
                                inst.getFullName(), s.getName(), s.getActualTypeArguments(), s.getComponentType().getFormalTypeParameters()));
                        */
                        //TODO change this after removing everything that is related to Java/JavaDSL
                    } else {
                        Log.debug(s.getComponentType().toString(), "ComponentType");
                        Log.debug(s.getComponentType().getFormalTypeParameters().toString(), "FormalTypeParameters");
                        exchangeGenerics(s, createMap(s.getComponentType().getFormalTypeParameters(),
                                s.getActualTypeArguments()));
                    }
                });
        Log.debug("See next lines", "Fixing Wrong Ports");
    }

    public EMAComponentInstanceSymbol build() {
        if (name.isPresent() && symbolReference.isPresent()) {
            EMAComponentInstanceSymbol sym =
                    new EMAComponentInstanceSymbol(this.name.get(),
                            this.symbolReference.get());

            //TODO add checks that port names and subcomponent names are unique
            if(!getPackageName().equals("")) {
                sym.setPackageName(getPackageName());
                sym.setFullName(getPackageName() + "." + this.name.get());
            }

            final MutableScope scope = (MutableScope) sym.getSpannedScope();
            resolvingFilters.stream().forEachOrdered(f -> scope.addResolver(f));

            ports.stream().forEachOrdered(p -> scope.add(EMAPortBuilder.instantiate(p, sym.getFullName()))); // must be cloned since we change it if it has generics
            connectors.stream().forEachOrdered(c -> scope.add(EMAConnectorBuilder.instantiate(c, sym.getFullName())));
            subComponents.stream().forEachOrdered(s -> scope.add(s));

            sym.setActualTypeArguments(actualTypeArguments.values().stream().collect(Collectors.toList()));
            sym.setResolutionDeclarationSymbols(resolutionDeclarationSymbols);
            sym.setParameters(parameters);
            sym.setArguments(arguments);
            exchangeGenerics(sym, actualTypeArguments);



            Log.debug(sym.toString(), "build end sym");
            return sym;
        }
        Log.error("not all parameters have been set before to build the expanded component instance symbol");
        throw new Error("not all parameters have been set before to build the expanded component instance symbol");
    }

    public EMAComponentInstanceBuilder addConnectorIfNameDoesNotExists(EMAConnectorSymbol connector) {
        List<String> existingConnectorSources = this.connectors.stream().map(c -> c.getSource())
                .collect(Collectors.toList());
        List<String> existingConnectorTargets = this.connectors.stream().map(c -> c.getTarget())
                .collect(Collectors.toList());
        if (!existingConnectorSources.contains(connector.getSource()) && !existingConnectorTargets.contains(connector.getTarget())) {
            this.addConnector(connector);
        }
        return this;
    }

    public EMAComponentInstanceBuilder addConnectorsIfNameDoesNotExists(Collection<EMAConnectorSymbol> connectors) {
        connectors.stream().forEach(this::addConnectorIfNameDoesNotExists);
        return this;
    }

    public EMAComponentInstanceBuilder addResolutionDeclarationSymbols(List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols) {
        for (ResolutionDeclarationSymbol symbol : resolutionDeclarationSymbols) {
            if (!this.resolutionDeclarationSymbols.contains(symbol)) {
                Log.info("name: "+symbol.getNameToResolve() +" astResolution: "+symbol.getASTResolution().toString(),"Added ResolutionDeclarationSymbol To EMAComponentInstanceBuilder");
                this.resolutionDeclarationSymbols.add(symbol);
            }
        }

        return this;
    }

    public List<EMAVariable> getParameters() {
        return parameters;
    }

    public EMAComponentInstanceBuilder addParameters(List<EMAVariable> parameters) {
        for (EMAVariable parameter : parameters) {
            if (!this.parameters.contains(parameter))
                this.parameters.add(parameter);
        }
        return this;
    }

    public List<ASTExpression> getArguments() {
        return arguments;
    }

    public EMAComponentInstanceBuilder addArguments(List<ASTExpression> arguments) {
        for (ASTExpression argument : arguments) {
            if (!this.arguments.contains(argument))
                this.arguments.add(argument);
        }
        return this;
    }

    public String getPackageName() {
        return packageName;
    }

    public EMAComponentInstanceBuilder setPackageName(String packageName) {
        this.packageName = packageName;
        return this;
    }
}
