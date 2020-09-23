/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.lang.embeddedmontiview.helper.SymbolPrinter;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

/**
 * Created by Michael von Wenckstern on 13.06.2016.
 *
 *         This class allows to modify {@see ViewComponentSymbol},
 *         if you do so the symbol table may not be consistent.
 *         Especially you need to call {@see EmbeddedMontiArcExpandedComponentInstanceSymbolCreator#createInstances}
 *         TODO static methods should call a protected doMethod() to allow extending this class
 *         TODO the builder should also be used to create a new ViewComponentSymbol with a build() method
 */
public class EMAComponentBuilder {
  protected static EMAComponentBuilder instance = null;

  protected static EMAComponentBuilder getInstance() {
    if (instance == null) {
      instance = new EMAComponentBuilder();
    }
    return instance;
  }

  public EMAComponentBuilder() {
  }

  private static final ResolvingFilter<ViewPortSymbol> portResolvingFilter = CommonResolvingFilter.create(ViewPortSymbol.KIND);

  private static final ResolvingFilter<ViewConnectorSymbol> connectorResolvingFilter = CommonResolvingFilter.create(ViewConnectorSymbol.KIND);

  private static final ResolvingFilter<ViewEffectorSymbol> effectorResolvingFilter = CommonResolvingFilter.create(ViewEffectorSymbol.KIND);

  private static final ResolvingFilter<ViewComponentSymbol> componentResolvingFilter = CommonResolvingFilter.create(ViewComponentSymbol.KIND);

  private static final ResolvingFilter<MCTypeSymbol> MCTypeSymbolResolvingGilter = CommonResolvingFilter.create(MCTypeSymbol.KIND);

  private static final ResolvingFilter<MCFieldSymbol> jAttributeResolvingFilter = CommonResolvingFilter.create(MCFieldSymbol.KIND);

  private static final ResolvingFilter<ViewComponentInstanceSymbol> componentInstanceResolvingFilter = CommonResolvingFilter.create(ViewComponentInstanceSymbol.KIND);

  ////////////////////////// ports //////////////////////////////////////////////

  public static EMAComponentBuilder addPort(ViewComponentSymbol cs, ViewPortSymbol ps) {
    if (!cs.getSpannedScope().getResolvingFilters().contains(portResolvingFilter)) {
      ((MutableScope) cs.getSpannedScope()).addResolver(portResolvingFilter);
    }
    ((MutableScope) cs.getSpannedScope()).add(ps);
    return getInstance();
  }

  public static EMAComponentBuilder addPorts(ViewComponentSymbol cs, ViewPortSymbol... ps) {
    for (ViewPortSymbol p : ps) {
      addPort(cs, p);
    }
    return getInstance();
  }

  public static EMAComponentBuilder addPorts(ViewComponentSymbol cs, Collection<ViewPortSymbol> ps) {
    ps.stream().forEachOrdered(p -> addPort(cs, p));
    return getInstance();
  }

  public static EMAComponentBuilder removePort(ViewComponentSymbol cs, ViewPortSymbol ps) {
    ((MutableScope) cs.getSpannedScope()).remove(ps);
    return getInstance();
  }

  public static EMAComponentBuilder removePorts(ViewComponentSymbol cs, ViewPortSymbol... ps) {
    for (ViewPortSymbol p : ps) {
      removePort(cs, p);
    }
    return getInstance();
  }

  public static EMAComponentBuilder removePorts(ViewComponentSymbol cs, Collection<ViewPortSymbol> ps) {
    ps.stream().forEachOrdered(p -> removePort(cs, p));
    return getInstance();
  }

  ////////////////////////// connectors //////////////////////////////////////////////

  public static EMAComponentBuilder addConnector(ViewComponentSymbol cs, ViewConnectorSymbol con) {
    if (!cs.getSpannedScope().getResolvingFilters().contains(connectorResolvingFilter)) {
      ((MutableScope) cs.getSpannedScope()).addResolver(connectorResolvingFilter);
    }
    ((MutableScope) cs.getSpannedScope()).add(con);
    return getInstance();
  }

  public static EMAComponentBuilder addEffector(ViewComponentSymbol cs, ViewEffectorSymbol eff) {
    if (!cs.getSpannedScope().getResolvingFilters().contains(effectorResolvingFilter)) {
      ((MutableScope) cs.getSpannedScope()).addResolver(effectorResolvingFilter);
    }
    ((MutableScope) cs.getSpannedScope()).add(eff);
    return getInstance();
  }

  public static EMAComponentBuilder addConnectors(ViewComponentSymbol cs, ViewConnectorSymbol... con) {
    for (ViewConnectorSymbol c : con) {
      addConnector(cs, c);
    }
    return getInstance();
  }

  public static EMAComponentBuilder addConnectors(ViewComponentSymbol cs, Collection<ViewConnectorSymbol> con) {
    con.stream().forEachOrdered(c -> addConnector(cs, c));
    return getInstance();
  }

  public static EMAComponentBuilder removeConnector(ViewComponentSymbol cs, ViewConnectorSymbol con) {
    ((MutableScope) cs.getSpannedScope()).remove(con);
    return getInstance();
  }

  public static EMAComponentBuilder removeConnectors(ViewComponentSymbol cs, ViewConnectorSymbol... con) {
    for (ViewConnectorSymbol c : con) {
      removeConnector(cs, c);
    }
    return getInstance();
  }

  public static EMAComponentBuilder removeConnectors(ViewComponentSymbol cs, Collection<ViewConnectorSymbol> con) {
    con.stream().forEachOrdered(c -> removeConnector(cs, c));
    return getInstance();
  }

  ////////////////////////// inner components //////////////////////////////////////////////

  public static EMAComponentBuilder addInnerComponent(ViewComponentSymbol cs, ViewComponentSymbol innerComponent) {
    if (!cs.getSpannedScope().getResolvingFilters().contains(componentResolvingFilter)) {
      ((MutableScope) cs.getSpannedScope()).addResolver(componentResolvingFilter);
    }
    ((MutableScope) cs.getSpannedScope()).add(innerComponent);
    return getInstance();
  }

  public static EMAComponentBuilder addInnerComponents(ViewComponentSymbol cs, ViewComponentSymbol... innerComponent) {
    for (ViewComponentSymbol c : innerComponent) {
      addInnerComponent(cs, c);
    }
    return getInstance();
  }

  public static EMAComponentBuilder addInnerComponents(ViewComponentSymbol cs, Collection<ViewComponentSymbol> innerComponent) {
    innerComponent.stream().forEachOrdered(c -> addInnerComponent(cs, c));
    return getInstance();
  }

  public static EMAComponentBuilder removeInnerComponent(ViewComponentSymbol cs, ViewComponentSymbol innerComponent) {
    ((MutableScope) cs.getSpannedScope()).remove(innerComponent);
    return getInstance();
  }

  public static EMAComponentBuilder removeInnerComponents(ViewComponentSymbol cs, ViewComponentSymbol... innerComponent) {
    for (ViewComponentSymbol c : innerComponent) {
      removeInnerComponent(cs, c);
    }
    return getInstance();
  }

  public static EMAComponentBuilder removeInnerComponents(ViewComponentSymbol cs, Collection<ViewComponentSymbol> innerComponent) {
    innerComponent.stream().forEachOrdered(c -> removeInnerComponent(cs, c));
    return getInstance();
  }

  ////////////////////////// formal type parameters //////////////////////////////////////////////

  public static EMAComponentBuilder addFormalTypeParameter(ViewComponentSymbol cs, MCTypeSymbol formalTypeParameter) {
    if (!formalTypeParameter.isFormalTypeParameter()) {
      Log.error(String.format("%s is not a formal type parameter. MCTypeSymbol#isFormalTypeParameter() is false.", SymbolPrinter.printFormalTypeParameters(formalTypeParameter)));
    }
    if (!cs.getSpannedScope().getResolvingFilters().contains(MCTypeSymbolResolvingGilter)) {
      ((MutableScope) cs.getSpannedScope()).addResolver(MCTypeSymbolResolvingGilter);
    }
    ((MutableScope) cs.getSpannedScope()).add(formalTypeParameter);
    return getInstance();
  }

  public static EMAComponentBuilder addFormalTypeParameters(ViewComponentSymbol cs, MCTypeSymbol... formalTypeParameter) {
    for (MCTypeSymbol f : formalTypeParameter) {
      addFormalTypeParameter(cs, f);
    }
    return getInstance();
  }

  public static EMAComponentBuilder addFormalTypeParameters(ViewComponentSymbol cs, Collection<MCTypeSymbol> formalTypeParameter) {
    formalTypeParameter.stream().forEachOrdered(f -> addFormalTypeParameter(cs, f));
    return getInstance();
  }

  public static EMAComponentBuilder removeFormalTypeParameter(ViewComponentSymbol cs, MCTypeSymbol formalTypeParameter) {
    ((MutableScope) cs.getSpannedScope()).remove(formalTypeParameter);
    return getInstance();
  }

  public static EMAComponentBuilder removeFormalTypeParameters(ViewComponentSymbol cs, MCTypeSymbol... formalTypeParameter) {
    for (MCTypeSymbol f : formalTypeParameter) {
      removeFormalTypeParameter(cs, f);
    }
    return getInstance();
  }

  public static EMAComponentBuilder removeFormalTypeParameters(ViewComponentSymbol cs, Collection<MCTypeSymbol> formalTypeParameter) {
    formalTypeParameter.stream().forEachOrdered(f -> removeFormalTypeParameter(cs, f));
    return getInstance();
  }

  ////////////////////////// config parameters //////////////////////////////////////////////

  public static EMAComponentBuilder addConfigParameter(ViewComponentSymbol cs, MCFieldSymbol configParameter) {
    if (!cs.getSpannedScope().getResolvingFilters().contains(jAttributeResolvingFilter)) {
      ((MutableScope) cs.getSpannedScope()).addResolver(jAttributeResolvingFilter);
    }
    ((MutableScope) cs.getSpannedScope()).add(configParameter);
    return getInstance();
  }

  public static EMAComponentBuilder addConfigParameters(ViewComponentSymbol cs, MCFieldSymbol... configParameter) {
    for (MCFieldSymbol c : configParameter) {
      addConfigParameter(cs, c);
    }
    return getInstance();
  }

  public static EMAComponentBuilder addConfigParameters(ViewComponentSymbol cs, Collection<MCFieldSymbol> configParameter) {
    configParameter.stream().forEachOrdered(c -> addConfigParameter(cs, c));
    return getInstance();
  }

  public static EMAComponentBuilder removeConfigParameter(ViewComponentSymbol cs, MCFieldSymbol configParameter) {
    ((MutableScope) cs.getSpannedScope()).remove(configParameter);
    return getInstance();
  }

  public static EMAComponentBuilder removeConfigParameters(ViewComponentSymbol cs, MCFieldSymbol... configParameter) {
    for (MCFieldSymbol c : configParameter) {
      removeConfigParameter(cs, c);
    }
    return getInstance();
  }

  public static EMAComponentBuilder removeConfigParameters(ViewComponentSymbol cs, Collection<MCFieldSymbol> configParameter) {
    configParameter.stream().forEachOrdered(c -> removeConfigParameter(cs, c));
    return getInstance();
  }

  ////////////////////////// sub components //////////////////////////////////////////////

  public static EMAComponentBuilder addSubComponent(ViewComponentSymbol cs, ViewComponentInstanceSymbol subComponent) {
    if (!cs.getSpannedScope().getResolvingFilters().contains(componentInstanceResolvingFilter)) {
      ((MutableScope) cs.getSpannedScope()).addResolver(componentInstanceResolvingFilter);
    }
    ((MutableScope) cs.getSpannedScope()).add(subComponent);
    return getInstance();
  }

  public static EMAComponentBuilder addSubComponents(ViewComponentSymbol cs, ViewComponentInstanceSymbol... subComponent) {
    for (ViewComponentInstanceSymbol s : subComponent) {
      addSubComponent(cs, s);
    }
    return getInstance();
  }

  public static EMAComponentBuilder addSubComponents(ViewComponentSymbol cs, Collection<ViewComponentInstanceSymbol> subComponent) {
    subComponent.stream().forEachOrdered(s -> addSubComponent(cs, s));
    return getInstance();
  }

  public static EMAComponentBuilder removeSubComponent(ViewComponentSymbol cs, ViewComponentInstanceSymbol subComponent) {
    ((MutableScope) cs.getSpannedScope()).remove(subComponent);
    return getInstance();
  }

  public static EMAComponentBuilder removeSubComponents(ViewComponentSymbol cs, ViewComponentInstanceSymbol... subComponent) {
    for (ViewComponentInstanceSymbol s : subComponent) {
      removeSubComponent(cs, s);
    }
    return getInstance();
  }

  public static EMAComponentBuilder removeSubComponents(ViewComponentSymbol cs, Collection<ViewComponentInstanceSymbol> subComponent) {
    subComponent.stream().forEachOrdered(s -> removeSubComponent(cs, s));
    return getInstance();
  }

}
