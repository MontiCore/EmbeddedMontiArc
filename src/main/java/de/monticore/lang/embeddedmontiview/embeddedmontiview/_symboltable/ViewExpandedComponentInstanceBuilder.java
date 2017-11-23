/**
 * ******************************************************************************
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
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Created by Michael von Wenckstern on 23.05.2016.
 */
public class ViewExpandedComponentInstanceBuilder {
  protected Optional<String> name = Optional.empty();
  protected Optional<ViewComponentSymbolReference> symbolReference = Optional.empty();
  protected List<ViewPortSymbol> ports = new ArrayList<>();
  protected List<ViewExpandedComponentInstanceSymbol> subComponents = new ArrayList<>();
  protected List<ViewConnectorSymbol> connectors = new ArrayList<>();
  protected Set<ResolvingFilter> resolvingFilters = new LinkedHashSet<>();
  //             FormalTypeParameter, ActualTypeArgument (is the binding of formal parameters
  protected Map<MCTypeSymbol, ActualTypeArgument> actualTypeArguments = new LinkedHashMap<>();

  protected static Map<MCTypeSymbol, ActualTypeArgument> createMap(List<MCTypeSymbol> keys, List<ActualTypeArgument> values) {
    Map<MCTypeSymbol, ActualTypeArgument> ret = new LinkedHashMap<>();
    for (int i = 0; i < keys.size(); i++) {
      ret.put(keys.get(i), values.get(i));
    }
    return ret;
  }

  public static ViewExpandedComponentInstanceSymbol clone(ViewExpandedComponentInstanceSymbol inst) {
    return new ViewExpandedComponentInstanceBuilder().setName(inst.getName()).setSymbolReference(inst.getComponentType())
        //.addPorts(inst.getPorts().stream().map(p -> EMAPortBuilder.clone(p)).collect(Collectors.toList()))
        .addPorts(inst.getPorts()) // is cloned in build method
        .addConnectors(inst.getConnectors().stream().map(c -> ViewConnectorBuilder.clone(c)).collect(Collectors.toList())).addSubComponents(inst.getSubComponents().stream().map(s -> ViewExpandedComponentInstanceBuilder.clone(s)).collect(Collectors.toList())).build();
  }

  public ViewExpandedComponentInstanceBuilder addResolvingFilter(ResolvingFilter filter) {
    this.resolvingFilters.add(filter);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addResolvingFilters(Set<ResolvingFilter<? extends Symbol>> filters) {
    for (ResolvingFilter filter : filters) {
      this.addResolvingFilter(filter);
    }
    return this;
  }

  public ViewExpandedComponentInstanceBuilder setName(String name) {
    this.name = Optional.of(name);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder setSymbolReference(ViewComponentSymbolReference symbolReference) {
    this.symbolReference = Optional.of(symbolReference);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addPort(ViewPortSymbol port) {
    this.ports.add(port);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addPorts(ViewPortSymbol... ports) {
    for (ViewPortSymbol p : ports) {
      this.addPort(p);
    }
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addPorts(Collection<ViewPortSymbol> ports) {
    ports.stream().forEachOrdered(p -> this.addPort(p));
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addActualTypeArgument(MCTypeSymbol formalTypeParameter, ActualTypeArgument typeArgument) {
    this.actualTypeArguments.put(formalTypeParameter, typeArgument);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addActualTypeArguments(List<MCTypeSymbol> formalTypeParameters, List<ActualTypeArgument> actualTypeArguments) {
    if (formalTypeParameters.size() != actualTypeArguments.size()) {
      Log.debug(formalTypeParameters.toString(), "FormalTypeParameters");
      Log.debug(actualTypeArguments.toString(), "ActualTypeArguments");
      Log.debug("instance has not as many actual type arguments as component definition has formal type parameters. No mapping is possible. Function does nothing.", ViewExpandedComponentInstanceBuilder.class.toString());
    }
    else {
      for (int i = 0; i < formalTypeParameters.size(); i++) {
        this.addActualTypeArgument(formalTypeParameters.get(i), actualTypeArguments.get(i));
      }
    }
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addPortsIfNameDoesNotExists(Collection<ViewPortSymbol> ports) {
    List<String> existingPortNames = this.ports.stream().map(p -> p.getName()).collect(Collectors.toList());
    this.addPorts(ports.stream().filter(p -> !existingPortNames.contains(p.getName())).collect(Collectors.toList()));
    return this;
  }

  /**
   * adds ports if they do not exist and replace generics of ports
   */
  public ViewExpandedComponentInstanceBuilder addPortsIfNameDoesNotExists(Collection<ViewPortSymbol> ports, List<MCTypeSymbol> formalTypeParameters, List<ActualTypeArgument> actualTypeArguments) {
    List<ViewPortSymbol> pList = ports.stream().collect(Collectors.toList());
    createMap(formalTypeParameters, actualTypeArguments).forEach((k, v) -> ports.stream().filter(p -> p.getTypeReference().get().getReferencedSymbol().getName().equals(k.getName())).forEachOrdered(p -> {
      ViewPortSymbol pCloned = EMAPortBuilder.clone(p);
      pCloned.setTypeReference(Optional.of((MCTypeReference<? extends MCTypeSymbol>) v.getType()));
      Collections.replaceAll(pList, p, pCloned);
    }));
    this.addPortsIfNameDoesNotExists(pList);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addSubComponent(ViewExpandedComponentInstanceSymbol subCmp) {
    this.subComponents.add(subCmp);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addSubComponentIfNameDoesNotExists(ViewExpandedComponentInstanceSymbol subCmp) {
    List<String> existingSubComponentNames = this.subComponents.stream().map(s -> s.getName()).collect(Collectors.toList());
    if (!existingSubComponentNames.contains(subCmp.getName())) {
      this.addSubComponent(subCmp);
    }
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addSubComponents(ViewExpandedComponentInstanceSymbol... subCmps) {
    for (ViewExpandedComponentInstanceSymbol s : subCmps) {
      this.addSubComponent(s);
    }
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addSubComponents(Collection<ViewExpandedComponentInstanceSymbol> subCmps) {
    subCmps.stream().forEachOrdered(s -> this.addSubComponent(s));
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addSubComponentsIfNameDoesNotExists(Collection<ViewExpandedComponentInstanceSymbol> subCmps) {
    List<String> existingSubComponentNames = this.subComponents.stream().map(s -> s.getName()).collect(Collectors.toList());
    this.addSubComponents(subCmps.stream().filter(s -> !existingSubComponentNames.contains(s.getName())).collect(Collectors.toList()));
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addConnector(ViewConnectorSymbol connector) {
    this.connectors.add(connector);
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addConnectors(ViewConnectorSymbol... connectors) {
    for (ViewConnectorSymbol c : connectors) {
      this.addConnector(c);
    }
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addConnectors(Collection<ViewConnectorSymbol> connectors) {
    connectors.stream().forEachOrdered(c -> this.addConnector(c));
    return this;
  }

  protected void exchangeGenerics(ViewExpandedComponentInstanceSymbol inst, Map<MCTypeSymbol, ActualTypeArgument> mapTypeArguments) {
    Log.debug(inst.toString(), "exchangeGenerics inst");
    // TODO work with full names, but then you got the problem with generics.GenericInstance.Generic.T != generics.SuperGenericComparableComp2.T
    // because when delegating the name of the referenced type must be created

    mapTypeArguments.forEach((k, v) -> {
      // 1) replace port generics
      inst.getPorts().stream()
          //          .filter(p -> p.getTypeReference().getReferencedSymbol().getFullName().equals(k.getFullName()))
          .filter(p -> p.getTypeReference().get().getReferencedSymbol().getName().equals(k.getName())).forEachOrdered(p -> p.setTypeReference(Optional.of((MCTypeReference<? extends MCTypeSymbol>) v.getType())));

      // 2) propagate component instance definition generics
      inst.getSubComponents().stream()
          // now update the actual type reference definitions by replacing them according to the hash map
          .forEachOrdered(s -> s.setActualTypeArguments(s.getActualTypeArguments().stream()
              // replace this filtered type argument with the value we want to replace
              //                  .map(a -> a.getType().getReferencedSymbol().getFullName().equals(k.getFullName()) ? v : a)
              .map(a -> a.getType().getReferencedSymbol().getName().equals(k.getName()) ? v : a).collect(Collectors.toList())));
    });

    // delegate generic exchanges through inner component hierarchy
    inst.getSubComponents().stream().forEachOrdered(s -> {
      if (s.getActualTypeArguments().size() != s.getComponentType().getFormalTypeParameters().size()) {
        Log.error(String.format("instance '%s' has a subcomponent instance '%s' where the given generics '%s' distinguish from the generics definition '%s'", inst.getFullName(), s.getName(), s.getActualTypeArguments(), s.getComponentType().getFormalTypeParameters()));
      }
      else {
        Log.debug(s.getComponentType().toString(), "ComponentType");
        Log.debug(s.getComponentType().getFormalTypeParameters().toString(), "FormalTypeParameters");
        exchangeGenerics(s, createMap(s.getComponentType().getFormalTypeParameters(), s.getActualTypeArguments()));
      }
    });
  }

  public ViewExpandedComponentInstanceSymbol build() {
    if (name.isPresent() && symbolReference.isPresent()) {
      ViewExpandedComponentInstanceSymbol sym = new ViewExpandedComponentInstanceSymbol(this.name.get(), this.symbolReference.get());

      //TODO add checks that port names and subcomponent names are unique
      final MutableScope scope = (MutableScope) sym.getSpannedScope();
      resolvingFilters.stream().forEachOrdered(f -> scope.addResolver(f));

      ports.stream().forEachOrdered(p -> scope.add(EMAPortBuilder.clone(p))); // must be cloned since we change it if it has generics
      connectors.stream().forEachOrdered(c -> scope.add(ViewConnectorBuilder.clone(c)));
      subComponents.stream().forEachOrdered(s -> scope.add(s));

      sym.setActualTypeArguments(actualTypeArguments.values().stream().collect(Collectors.toList()));
      exchangeGenerics(sym, actualTypeArguments);
      Log.debug(sym.toString(), "build end sym");
      return sym;
    }
    Log.error("not all parameters have been set before to build the expanded component instance symbol");
    throw new Error("not all parameters have been set before to build the expanded component instance symbol");
  }

  public ViewExpandedComponentInstanceBuilder addConnectorIfNameDoesNotExists(ViewConnectorSymbol connector) {
    List<String> existingConnectorSources = this.connectors.stream().map(c -> c.getSource()).collect(Collectors.toList());
    List<String> existingConnectorTargets = this.connectors.stream().map(c -> c.getTarget()).collect(Collectors.toList());
    if (!existingConnectorSources.contains(connector.getSource()) && !existingConnectorTargets.contains(connector.getTarget())) {
      this.addConnector(connector);
    }
    return this;
  }

  public ViewExpandedComponentInstanceBuilder addConnectorsIfNameDoesNotExists(Collection<ViewConnectorSymbol> connectors) {
    connectors.stream().forEach(this::addConnectorIfNameDoesNotExists);
    return this;
  }
}
