/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.helper.SymbolPrinter;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

/**
 * Created by Michael von Wenckstern on 13.06.2016.
 *
 *         This class allows to modify {@see EMAComponentSymbol},
 *         if you do so the symbol table may not be consistent.
 *         Especially you need to call {@see EMAComponentInstanceSymbolCreator#createInstances}
 *         TODO static methods should call a protected doMethod() to allow extending this class
 *         TODO the builder should also be used to create a new EMAComponentSymbol with a build() method
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

    private static final ResolvingFilter<EMAPortSymbol> portResolvingFilter =
            CommonResolvingFilter.create(EMAPortSymbol.KIND);

    private static final ResolvingFilter<EMAConnectorSymbol> connectorResolvingFilter =
            CommonResolvingFilter.create(EMAConnectorSymbol.KIND);

    private static final ResolvingFilter<EMAComponentSymbol> componentResolvingFilter =
            CommonResolvingFilter.create(EMAComponentSymbol.KIND);

    private static final ResolvingFilter<MCTypeSymbol> jTypeSymbolResolvingGilter =
            CommonResolvingFilter.create(MCTypeSymbol.KIND);

    private static final ResolvingFilter<MCFieldSymbol> jAttributeResolvingFilter =
            CommonResolvingFilter.create(MCFieldSymbol.KIND);

    private static final ResolvingFilter<EMAComponentInstantiationSymbol> emaComponentInstantiationResolvingFilter =
            CommonResolvingFilter.create(EMAComponentInstantiationSymbol.KIND);

    ////////////////////////// ports //////////////////////////////////////////////

    public static EMAComponentBuilder addPort(EMAComponentSymbol cs, EMAPortSymbol ps) {
        addResolverIfMissing(cs, portResolvingFilter, ps);
        return getInstance();
    }

    public static void addResolverIfMissing(EMAComponentSymbol cs, ResolvingFilter resolvingFilter, Symbol symbol) {
        if (!cs.getSpannedScope().getResolvingFilters().contains(resolvingFilter)) {
            ((MutableScope) cs.getSpannedScope()).addResolver(resolvingFilter);
        }
        ((MutableScope) cs.getSpannedScope()).add(symbol);
    }

    public static EMAComponentBuilder addPorts(EMAComponentSymbol cs, EMAPortSymbol... ps) {
        for (EMAPortSymbol p : ps) {
            addPort(cs, p);
        }
        return getInstance();
    }

    public static EMAComponentBuilder addPorts(EMAComponentSymbol cs, Collection<EMAPortSymbol> ps) {
        ps.stream().forEachOrdered(p -> addPort(cs, p));
        return getInstance();
    }

    public static EMAComponentBuilder removePort(EMAComponentSymbol cs, EMAPortSymbol ps) {
        ((MutableScope) cs.getSpannedScope()).remove(ps);
        return getInstance();
    }

    public static EMAComponentBuilder removePorts(EMAComponentSymbol cs, EMAPortSymbol... ps) {
        for (EMAPortSymbol p : ps) {
            removePort(cs, p);
        }
        return getInstance();
    }

    public static EMAComponentBuilder removePorts(EMAComponentSymbol cs, Collection<EMAPortSymbol> ps) {
        ps.stream().forEachOrdered(p -> removePort(cs, p));
        return getInstance();
    }

    ////////////////////////// connectors //////////////////////////////////////////////

    public static EMAComponentBuilder addConnector(EMAComponentSymbol cs, EMAConnectorSymbol con) {
        addResolverIfMissing(cs, connectorResolvingFilter, con);
        return getInstance();
    }

    public static EMAComponentBuilder addConnectors(EMAComponentSymbol cs, EMAConnectorSymbol... con) {
        for (EMAConnectorSymbol c : con) {
            addConnector(cs, c);
        }
        return getInstance();
    }

    public static EMAComponentBuilder addConnectors(EMAComponentSymbol cs, Collection<EMAConnectorSymbol> con) {
        con.stream().forEachOrdered(c -> addConnector(cs, c));
        return getInstance();
    }

    public static EMAComponentBuilder removeConnector(EMAComponentSymbol cs, EMAConnectorSymbol con) {
        ((MutableScope) cs.getSpannedScope()).remove(con);
        return getInstance();
    }

    public static EMAComponentBuilder removeConnectors(EMAComponentSymbol cs, EMAConnectorSymbol... con) {
        for (EMAConnectorSymbol c : con) {
            removeConnector(cs, c);
        }
        return getInstance();
    }

    public static EMAComponentBuilder removeConnectors(EMAComponentSymbol cs, Collection<EMAConnectorSymbol> con) {
        con.stream().forEachOrdered(c -> removeConnector(cs, c));
        return getInstance();
    }

    ////////////////////////// inner components //////////////////////////////////////////////

    public static EMAComponentBuilder addInnerComponent(EMAComponentSymbol cs, EMAComponentSymbol innerComponent) {
        addResolverIfMissing(cs, componentResolvingFilter, innerComponent);
        return getInstance();
    }

    public static EMAComponentBuilder addInnerComponents(EMAComponentSymbol cs, EMAComponentSymbol... innerComponent) {
        for (EMAComponentSymbol c : innerComponent) {
            addInnerComponent(cs, c);
        }
        return getInstance();
    }

    public static EMAComponentBuilder addInnerComponents(EMAComponentSymbol cs, Collection<EMAComponentSymbol> innerComponent) {
        innerComponent.stream().forEachOrdered(c -> addInnerComponent(cs, c));
        return getInstance();
    }

    public static EMAComponentBuilder removeInnerComponent(EMAComponentSymbol cs, EMAComponentSymbol innerComponent) {
        ((MutableScope) cs.getSpannedScope()).remove(innerComponent);
        return getInstance();
    }

    public static EMAComponentBuilder removeInnerComponents(EMAComponentSymbol cs, EMAComponentSymbol... innerComponent) {
        for (EMAComponentSymbol c : innerComponent) {
            removeInnerComponent(cs, c);
        }
        return getInstance();
    }

    public static EMAComponentBuilder removeInnerComponents(EMAComponentSymbol cs, Collection<EMAComponentSymbol> innerComponent) {
        innerComponent.stream().forEachOrdered(c -> removeInnerComponent(cs, c));
        return getInstance();
    }

    ////////////////////////// formal type parameters //////////////////////////////////////////////

    public static EMAComponentBuilder addFormalTypeParameter(EMAComponentSymbol cs, MCTypeSymbol formalTypeParameter) {
        if (!formalTypeParameter.isFormalTypeParameter()) {
            Log.error(String.format("%s is not a formal type parameter. MCTypeSymbol#isFormalTypeParameter() is false.",
                    SymbolPrinter.printFormalTypeParameters(formalTypeParameter)));
        }
        addResolverIfMissing(cs, jTypeSymbolResolvingGilter, formalTypeParameter);
        return getInstance();
    }

    public static EMAComponentBuilder addFormalTypeParameters(EMAComponentSymbol cs, MCTypeSymbol... formalTypeParameter) {
        for (MCTypeSymbol f : formalTypeParameter) {
            addFormalTypeParameter(cs, f);
        }
        return getInstance();
    }

    public static EMAComponentBuilder addFormalTypeParameters(EMAComponentSymbol cs, Collection<MCTypeSymbol> formalTypeParameter) {
        formalTypeParameter.stream().forEachOrdered(f -> addFormalTypeParameter(cs, f));
        return getInstance();
    }

    public static EMAComponentBuilder removeFormalTypeParameter(EMAComponentSymbol cs, MCTypeSymbol formalTypeParameter) {
        ((MutableScope) cs.getSpannedScope()).remove(formalTypeParameter);
        return getInstance();
    }

    public static EMAComponentBuilder removeFormalTypeParameters(EMAComponentSymbol cs, MCTypeSymbol... formalTypeParameter) {
        for (MCTypeSymbol f : formalTypeParameter) {
            removeFormalTypeParameter(cs, f);
        }
        return getInstance();
    }

    public static EMAComponentBuilder removeFormalTypeParameters(EMAComponentSymbol cs, Collection<MCTypeSymbol> formalTypeParameter) {
        formalTypeParameter.stream().forEachOrdered(f -> removeFormalTypeParameter(cs, f));
        return getInstance();
    }

    ////////////////////////// config parameters //////////////////////////////////////////////

    public static EMAComponentBuilder addConfigParameter(EMAComponentSymbol cs, MCFieldSymbol configParameter) {
        addResolverIfMissing(cs, jAttributeResolvingFilter, configParameter);
        return getInstance();
    }

    public static EMAComponentBuilder addConfigParameters(EMAComponentSymbol cs, MCFieldSymbol... configParameter) {
        for (MCFieldSymbol c : configParameter) {
            addConfigParameter(cs, c);
        }
        return getInstance();
    }

    public static EMAComponentBuilder addConfigParameters(EMAComponentSymbol cs, Collection<MCFieldSymbol> configParameter) {
        configParameter.stream().forEachOrdered(c -> addConfigParameter(cs, c));
        return getInstance();
    }

    public static EMAComponentBuilder removeConfigParameter(EMAComponentSymbol cs, MCFieldSymbol configParameter) {
        ((MutableScope) cs.getSpannedScope()).remove(configParameter);
        return getInstance();
    }

    public static EMAComponentBuilder removeConfigParameters(EMAComponentSymbol cs, MCFieldSymbol... configParameter) {
        for (MCFieldSymbol c : configParameter) {
            removeConfigParameter(cs, c);
        }
        return getInstance();
    }

    public static EMAComponentBuilder removeConfigParameters(EMAComponentSymbol cs, Collection<MCFieldSymbol> configParameter) {
        configParameter.stream().forEachOrdered(c -> removeConfigParameter(cs, c));
        return getInstance();
    }

    ////////////////////////// sub components //////////////////////////////////////////////

    public static EMAComponentBuilder addSubComponent(EMAComponentSymbol cs, EMAComponentInstantiationSymbol subComponent) {
        addResolverIfMissing(cs, emaComponentInstantiationResolvingFilter, subComponent);
        return getInstance();
    }

    public static EMAComponentBuilder addSubComponents(EMAComponentSymbol cs, EMAComponentInstantiationSymbol... subComponent) {
        for (EMAComponentInstantiationSymbol s : subComponent) {
            addSubComponent(cs, s);
        }
        return getInstance();
    }

    public static EMAComponentBuilder addSubComponents(EMAComponentSymbol cs, Collection<EMAComponentInstantiationSymbol> subComponent) {
        subComponent.stream().forEachOrdered(s -> addSubComponent(cs, s));
        return getInstance();
    }

    public static EMAComponentBuilder removeSubComponent(EMAComponentSymbol cs, EMAComponentInstantiationSymbol subComponent) {
        ((MutableScope) cs.getSpannedScope()).remove(subComponent);
        return getInstance();
    }

    public static EMAComponentBuilder removeSubComponents(EMAComponentSymbol cs, EMAComponentInstantiationSymbol... subComponent) {
        for (EMAComponentInstantiationSymbol s : subComponent) {
            removeSubComponent(cs, s);
        }
        return getInstance();
    }

    public static EMAComponentBuilder removeSubComponents(EMAComponentSymbol cs, Collection<EMAComponentInstantiationSymbol> subComponent) {
        subComponent.stream().forEachOrdered(s -> removeSubComponent(cs, s));
        return getInstance();
    }

}
