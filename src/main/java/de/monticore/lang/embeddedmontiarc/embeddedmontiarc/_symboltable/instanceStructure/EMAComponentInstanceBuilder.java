/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponentModifier;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPortInitial;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.EmbeddedMontiArcMill;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.UnitNumberExpressionSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.EMAVariable;
import de.monticore.lang.monticar.common2._ast.ASTLiteralValue;
import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.common2._ast.ASTValue;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberExpression;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolutionExpression;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.literals.literals._ast.*;
import de.monticore.numberunit._ast.ASTNumberWithInf;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.*;
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
    protected List<ASTPortInitial> portInitials = new ArrayList<>();
    protected List<ASTComponentModifier> componentModifiers = new ArrayList<>();

    protected static Map<MCTypeSymbol, ActualTypeArgument> createMap(List<MCTypeSymbol> keys,
                                                                     List<ActualTypeArgument> values) {
        Map<MCTypeSymbol, ActualTypeArgument> ret = new LinkedHashMap<>();
        for (int i = 0; i < keys.size(); i++) {
            ret.put(keys.get(i), values.get(i));
        }
        return ret;
    }

    public static EMAComponentInstanceSymbol clone(EMAComponentInstanceSymbol inst) {
        Collection<EMAComponentInstanceSymbol> subcomps =
                inst.getSubComponents().stream().map(EMAComponentInstanceBuilder::clone).collect(Collectors.toList());
        Collection<EMAConnectorSymbol> connectors =
                inst.getConnectorInstances().stream().map(EMAConnectorBuilder::clone).collect(Collectors.toList());
        Collection<EMAPortSymbol> ports =
                inst.getPortInstanceList().stream().map(EMAPortBuilder::clone).collect(Collectors.toList());

        EMAComponentInstanceBuilder res = (new EMAComponentInstanceBuilder());


        res
                .setName(inst.getName())
                .setSymbolReference(inst.getComponentType())
                .addPorts(ports)
                .addSubComponents(subcomps)
                .addConnectors(connectors)
                .addResolvingFilters(inst.getSpannedScope().getResolvingFilters())
                .addActualTypeArguments(inst.getComponentType().getFormalTypeParameters(),
                        inst.getActualTypeArguments())
                .addResolutionDeclarationSymbols(inst.getResolutionDeclarationSymbols())
                .addParameters(inst.getParameters())
                .addArguments(inst.getArguments())
                .addPortInitials(inst.getComponentType().getPortInitials())
                .addComponentModifiers(inst.getComponentModifiers())
                .setPackageName(inst.getPackageName());

        return res.build();
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

    public EMAComponentInstanceBuilder addActualTypeArgument(MCTypeSymbol formalTypeParameter,
                                                             ActualTypeArgument typeArgument) {
        this.actualTypeArguments.put(formalTypeParameter, typeArgument);
        return this;
    }

    public EMAComponentInstanceBuilder addActualTypeArguments(List<MCTypeSymbol> formalTypeParameters,
                                                              List<ActualTypeArgument> actualTypeArguments) {
        if (formalTypeParameters.size() != actualTypeArguments.size()) {
            Log.debug(formalTypeParameters.toString(), "FormalTypeParameters");
            Log.debug(actualTypeArguments.toString(), "ActualTypeArguments");
            Log.debug(
                    "instance has not as many actual type arguments as component definition has formal type parameters. No mapping is possible. Function does nothing.",
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
    public EMAComponentInstanceBuilder addPortsIfNameDoesNotExists(Collection<EMAPortSymbol> ports,
                                                                   List<MCTypeSymbol> formalTypeParameters, List<ActualTypeArgument> actualTypeArguments) {
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

    public EMAComponentInstanceBuilder addSubComponentsIfNameDoesNotExists(
            Collection<EMAComponentInstanceSymbol> subCmps) {
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
                    .filter(p -> p.getTypeReference().existsReferencedSymbol() ? p.getTypeReference()
                            .getReferencedSymbol()
                            .getName().equals(k.getName()) : false)
                    .forEachOrdered(p -> p.setTypeReference((MCTypeReference<? extends MCTypeSymbol>) v.getType()));

            // 2) propagate component instance definition generics
            inst.getSubComponents().stream()
                    // now update the actual type reference definitions by replacing them according to the hash map
                    .forEachOrdered(
                            s -> s.setActualTypeArguments(
                                    s.getActualTypeArguments().stream()
                                            // replace this filtered type arguments with the value we want to replace
                                            //                  .map(a -> a.getType().getReferencedSymbol().getFullName().equals(k.getFullName()) ? v : a)
                                            .map(a -> (a.getType().existsReferencedSymbol() ? (
                                                    a.getType().getReferencedSymbol().getName().equals(k.getName()) ? v : a)
                                                    : a))
                                            .collect(Collectors.toList())
                            ));

            // 3) replace Parameter generics
            // TODO more, maybe ranges etc. Parameter types should have the same class as all other types...
            inst.getParameters().stream()
                    .filter(emaVariable -> emaVariable.getType() instanceof ASTElementType
                                    && ((ASTElementType) emaVariable.getType()).isPresentName()
                                    && ((ASTElementType) emaVariable.getType()).getName().equals(k.getName()))
                    .forEach(emaVariable -> {
                        ((ASTElementType) emaVariable.getType()).setName(v.getType().getName());
                    });

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
                        //Hack to pass through replaced type symbols
                        Map<MCTypeSymbol, ActualTypeArgument> map =
                                createMap(s.getComponentType().getFormalTypeParameters(), s.getActualTypeArguments());
                        map.putAll(mapTypeArguments);
                        exchangeGenerics(s, map);
                    }
                });
        Log.debug("See next lines", "Fixing Wrong Ports");
    }

    protected void exchangeParameters(EMAComponentInstanceSymbol sym) {
        Map<String, ASTExpression> argumentsMap = new HashMap<>();
        for (int i = 0; i < parameters.size(); i++) {
            argumentsMap.put(parameters.get(i).getName(), arguments.get(i));
        }
        exchangeParameters(sym, argumentsMap);
    }

    protected void exchangeParameters(EMAComponentInstanceSymbol inst, Map<String, ASTExpression> arguments) {
        for (EMAComponentInstanceSymbol subComponent : inst.getSubComponents()) {
            for (int i = 0; i < subComponent.getArguments().size(); i++) {
                ASTExpression argument = subComponent.getArguments().get(i);
                ASTExpression exchange = calculateExchange(argument, arguments);

                if (exchange != null) {
                    subComponent.getArguments().set(i, exchange);
                }
            }
            if (subComponent.getParameters().size() == subComponent.getArguments().size()) {
                for (int i = 0; i < subComponent.getParameters().size(); i++) {
                    arguments.put(subComponent.getParameters().get(i).getName(), subComponent.getArguments().get(i));
                }
                exchangeParameters(subComponent, arguments);
            } else if (!subComponent.getArguments().isEmpty() &&
                    subComponent.getArguments().size() != subComponent.getParameters().size()) {
                Log.error("TODO");
            }
        }

        for (EMAPortInstanceSymbol port : inst.getPortInstanceList()) {
            if (port.isInitialGuessPresent() || port.isInitialValuePresent()) {
                ASTExpression initial = port.isInitialGuessPresent() ? port.getInitialGuess() : port.getInitialValue();
                if (initial instanceof ASTUnitNumberResolutionExpression) {
                    if (((ASTUnitNumberResolutionExpression) initial).getUnitNumberResolution().getNameOpt().isPresent()) {
                        String par = ((ASTUnitNumberResolutionExpression) initial).getUnitNumberResolution().getName();
                        ASTExpression argument = arguments.get(par);
                        if (argument != null) {
                            if (port.isInitialGuessPresent())
                                port.setInitialGuess(argument);
                            else
                                port.setInitialValue(argument);
                        }
                    }
                }
            }
        }
    }

    protected ASTExpression calculateExchange(ASTExpression argument, Map<String, ASTExpression> arguments) {
        if (argument instanceof ASTUnitNumberResolutionExpression) {
            if (((ASTUnitNumberResolutionExpression) argument).getUnitNumberResolution().isPresentName()) {
                String argumentName =
                        ((ASTUnitNumberResolutionExpression) argument).getUnitNumberResolution().getName();
                return arguments.get(argumentName);
            } else {
                // TODO Fall untersuchen
            }
        }
        // TODO Fall untersuchen
        return null;
    }

    public void addPortArraySymbolsToInstance(EMAComponentInstanceSymbol instance) {
        Map<String, List<EMAPortInstanceSymbol>> nameToPortList = new HashMap<>();
        for (EMAPortInstanceSymbol port : instance.getPortInstanceList()) {
            List<EMAPortInstanceSymbol> list = nameToPortList
                    .computeIfAbsent(port.getNameWithoutArrayBracketPart(), k -> new ArrayList<>());
            list.add(port);
            if (list.size() > 1) {
                list.toString();
            }
        }

        for (String name : nameToPortList.keySet()) {
            if (!instance.getSpannedScope().resolveLocally(name, EMAPortArraySymbol.KIND).isPresent()) {
                List<EMAPortInstanceSymbol> ports = nameToPortList.get(name);
                EMAPortArraySymbol portArray = new EMAPortArraySymbol(name, null);
                portArray.setDimension(ports.size());
                portArray.setDirection(ports.get(0).isIncoming());
                portArray.setTypeReference(ports.get(0).getTypeReference());
                instance.getSpannedScope().getAsMutableScope().add(portArray);
            }
        }
    }

    public EMAComponentInstanceSymbol build() {
        if (name.isPresent() && symbolReference.isPresent()) {
            EMAComponentInstanceSymbol sym = instantiateComponentSymbol();

            //TODO add checks that port names and subcomponent names are unique
            if (!getPackageName().equals("")) {
                sym.setPackageName(getPackageName());
                sym.setFullName(getPackageName() + "." + this.name.get());
            }

            final MutableScope scope = (MutableScope) sym.getSpannedScope();
            resolvingFilters.stream().forEachOrdered(f -> scope.addResolver(f));

            String componentFullName = Names.getQualifiedName(sym.getPackageName(), sym.getName());
            ports.stream().forEachOrdered(p ->
                    handlePort(p, componentFullName, scope)); // must be cloned since we change it if it has
            addPortArraySymbolsToInstance(sym);
            handlePortInitials(sym);

            // Component Modifiers
            sym.setComponentModifiers(componentModifiers);

            // generics
            connectors.stream().forEachOrdered(c -> instantiateConnectorSymbol(c, componentFullName, scope));
            subComponents.stream().forEachOrdered(s -> scope.add(s));

            sym.setActualTypeArguments(actualTypeArguments.values().stream().collect(Collectors.toList()));
            sym.setResolutionDeclarationSymbols(resolutionDeclarationSymbols);
            sym.setParameters(parameters);
            addOtherToComponentInstance(sym);

            // set arguments
            // there are either no arguments or the equal number to parameters
            if (!arguments.isEmpty() && arguments.size() != parameters.size()) {
                Log.error("TODO Wrong number of arguments: " + componentFullName);
            }
            setDefaultValuesToArguments(sym);
            sym.setArguments(arguments);
            exchangeGenerics(sym, actualTypeArguments);

            if (parameters.size() == arguments.size()) {
                exchangeParameters(sym);
            }

            Log.debug(sym.toString(), "build end sym");
            return sym;
        }
        Log.error("not all parameters have been set before to build the expanded component instance symbol");
        throw new Error("not all parameters have been set before to build the expanded component instance symbol");
    }

    private void handlePortInitials(EMAComponentInstanceSymbol sym) {
        Collection<EMAPortInstanceSymbol> portInstanceList = sym.getPortInstanceList();
        for (ASTPortInitial initialGuess : portInitials) {
            String arrayAccess = "";
            if (initialGuess.isPresentUnitNumberResolution())
                arrayAccess += "[" + initialGuess.getUnitNumberResolution().getNumber().get().intValue() + "]";
            final String portAccessName = initialGuess.getName() + arrayAccess;
            portInstanceList.stream()
                    .filter(port -> port.getName().equals(portAccessName))
                    .forEachOrdered(port -> {
                        if (initialGuess.isGuess())
                            port.setInitialGuess(initialGuess.getExpression());
                        else
                            port.setInitialValue(initialGuess.getExpression());
                    });
        }
    }

    private void setDefaultValuesToArguments(EMAComponentInstanceSymbol sym) {
        if (arguments.isEmpty() && !parameters.isEmpty()) {
            // set default values
            for (ASTParameter astParameter :
                    ((ASTComponent) sym.getComponentType().getReferencedComponent().get().getAstNode().get())
                            .getParameterList()) {
                if (astParameter.isPresentDefaultValue()) {
                    ASTExpression argument = createArgumentFromDefaultValue(astParameter);
                    arguments.add(argument);
                } else {
                    // TODO Do nothing for default instances else log error
//                        Log.error("TODO No default value given for parameter with missing argument: "
//                            + sym.getFullName() + "." + astParameter.getNameWithArray().getName());
                }
            }
        }
    }

    protected EMAComponentInstanceSymbol instantiateComponentSymbol() {
        return new EMAComponentInstanceSymbol(this.name.get(),
                this.symbolReference.get());
    }

    protected void instantiateConnectorSymbol(EMAConnectorSymbol c, String fullName,
                                              MutableScope scope) {
        scope.add(EMAConnectorBuilder.instantiate(c, fullName));
    }

    protected void handlePort(EMAPortSymbol port, String packageName, MutableScope scope) {
        if (port instanceof EMAPortArraySymbol)
            instantiatePortArraySymbol((EMAPortArraySymbol) port, packageName, scope);
        else
            instantiatePortSymbol(port, packageName, port.getName(), scope);
    }

    protected EMAPortInstanceSymbol instantiatePortSymbol(EMAPortSymbol port, String packageName, String name, MutableScope scope) {
        EMAPortInstanceSymbol symbol = EMAPortBuilder.instantiate(port, packageName, name);
        scope.add(symbol);
        return symbol;
    }

    protected void instantiatePortArraySymbol(EMAPortArraySymbol port, String packageName, MutableScope scope) {
        for (int i = 0; i < port.getDimension(); ++i) {
            String portName = port.getName() + "[" + (i + 1) + "]";
            instantiatePortSymbol(port, packageName, portName, scope);
        }
    }

    protected void addOtherToComponentInstance(EMAComponentInstanceSymbol sym) {
        // can be overriden
    }

    protected ASTExpression createArgumentFromDefaultValue(ASTParameter astParameter) {
        ASTValue defaultValue = astParameter.getDefaultValue();
        if (defaultValue instanceof ASTLiteralValue) {
            if (((ASTLiteralValue) defaultValue).getValue() instanceof ASTSignedNumericLiteral) {
                ASTNumberWithInf numberWithInf =
                        createNumberWithInfFromLiteralValue(
                                (ASTSignedNumericLiteral) ((ASTLiteralValue) defaultValue).getValue());
                ASTNumberWithUnit numberWithUnit =
                        EmbeddedMontiArcMill
                                .numberWithUnitBuilder()
                                .setNum(numberWithInf)
                                .build();
                ASTUnitNumberExpression expression =
                        EmbeddedMontiArcMill
                                .unitNumberExpressionBuilder()
                                .setNumberWithUnit(numberWithUnit)
                                .build();
                UnitNumberExpressionSymbol symbol = new UnitNumberExpressionSymbol(expression);
                expression.setSymbol(symbol);
                return expression;
            } else if (((ASTLiteralValue) defaultValue).getValue() instanceof ASTBooleanLiteral) {
                return EmbeddedMontiArcMill
                        .booleanExpressionBuilder()
                        .setBooleanLiteral((ASTBooleanLiteral) ((ASTLiteralValue) defaultValue).getValue())
                        .build();
            } else {
                Log.error("not supported parameter ASTCharLiteral, ASTStringLiteral, ASTNullLIteral");
            }
        } else {
            Log.error("not supported ASTValue implementation");
        }
        return null;
    }

    protected ASTNumberWithInf createNumberWithInfFromLiteralValue(ASTSignedNumericLiteral defaultValue) {
        ASTNumericLiteral literal = null;
        String negNumber = null;
        if (defaultValue instanceof ASTSignedDoubleLiteral) {
            if (((ASTSignedDoubleLiteral) defaultValue).isNegative())
                negNumber = "-";
            literal = EmbeddedMontiArcMill
                    .intLiteralBuilder()
                    .setSource(((ASTSignedDoubleLiteral) defaultValue).getSource())
                    .build();
        } else if (defaultValue instanceof ASTSignedFloatLiteral) {
            if (((ASTSignedFloatLiteral) defaultValue).isNegative())
                negNumber = "-";
            literal = EmbeddedMontiArcMill
                    .intLiteralBuilder()
                    .setSource(((ASTSignedFloatLiteral) defaultValue).getSource())
                    .build();
        } else if (defaultValue instanceof ASTSignedIntLiteral) {
            if (((ASTSignedIntLiteral) defaultValue).isNegative())
                negNumber = "-";
            literal = EmbeddedMontiArcMill
                    .intLiteralBuilder()
                    .setSource(((ASTSignedIntLiteral) defaultValue).getSource())
                    .build();
        } else if (defaultValue instanceof ASTSignedLongLiteral) {
            if (((ASTSignedLongLiteral) defaultValue).isNegative())
                negNumber = "-";
            literal = EmbeddedMontiArcMill
                    .intLiteralBuilder()
                    .setSource(((ASTSignedLongLiteral) defaultValue).getSource())
                    .build();
        }
        return EmbeddedMontiArcMill
                .numberWithInfBuilder()
                .setNegNumber(negNumber)
                .setNumber(literal)
                .build();
    }

    public EMAComponentInstanceBuilder addConnectorIfNameDoesNotExists(EMAConnectorSymbol connector) {
        List<String> existingConnectorSources = this.connectors.stream().map(c -> c.getSource())
                .collect(Collectors.toList());
        List<String> existingConnectorTargets = this.connectors.stream().map(c -> c.getTarget())
                .collect(Collectors.toList());
        if (!existingConnectorSources.contains(connector.getSource()) &&
                !existingConnectorTargets.contains(connector.getTarget())) {
            this.addConnector(connector);
        }
        return this;
    }

    public EMAComponentInstanceBuilder addConnectorsIfNameDoesNotExists(Collection<EMAConnectorSymbol> connectors) {
        connectors.stream().forEach(this::addConnectorIfNameDoesNotExists);
        return this;
    }

    public EMAComponentInstanceBuilder addResolutionDeclarationSymbols(
            List<ResolutionDeclarationSymbol> resolutionDeclarationSymbols) {
        for (ResolutionDeclarationSymbol symbol : resolutionDeclarationSymbols) {
            if (!this.resolutionDeclarationSymbols.contains(symbol)) {
                Log.info("name: " + symbol.getNameToResolve() + " astResolution: " +
                                symbol.getASTResolution().toString(),
                        "Added ResolutionDeclarationSymbol To EMAComponentInstanceBuilder");
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

    public EMAComponentInstanceBuilder addPortInitials(List<ASTPortInitial> portInitials) {
        for (ASTPortInitial portInitial : portInitials) {
            if (!this.portInitials.contains(portInitial))
                this.portInitials.add(portInitial);
        }
        return this;
    }

    public List<ASTPortInitial> getPortInitials() {
        return portInitials;
    }

    public EMAComponentInstanceBuilder setPortInitials(List<ASTPortInitial> portInitials) {
        this.portInitials = portInitials;
        return this;
    }

    public List<ASTComponentModifier> getComponentModifiers() {
        return componentModifiers;
    }

    public EMAComponentInstanceBuilder addComponentModifiers(List<ASTComponentModifier> componentModifiers) {
        for (ASTComponentModifier componentModifier : componentModifiers) {
            if (!this.componentModifiers.contains(componentModifier))
                this.componentModifiers.add(componentModifier);
        }
        return this;
    }

    public EMAComponentInstanceBuilder setComponentModifiers(List<ASTComponentModifier> componentModifiers) {
        this.componentModifiers = componentModifiers;
        return this;
    }

    public void fixSubComponentPackageNames() {
        subComponents.stream().forEach(s -> {
            String newPackageName = Joiners.DOT.join(packageName, name.get());
            s.setPackageName(newPackageName);
            s.resetFullNames();
        });
    }
}
