/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTQualifiedNameWithArrayAndStar;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.monticar.common2._ast.ASTArrayAccess;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.symboltable.MutableScope;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class EMAPortHelper {
    /**
     * creates the PortSymbols that belong to a EMAPortArraySymbol
     */
    public static void portCreationIntLiteralPresent(ASTPort node, EMAPortArraySymbol pas, String name,
                                                     MCTypeReference<? extends MCTypeSymbol> typeRef,
                                                     EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        // int num = node.getIntLiteral().getValue();
        Log.debug(node.toString(), "ASTPort");
        int num = 0;
        if (node.getUnitNumberResolutionOpt().isPresent()
                && node.getUnitNumberResolution().getNumberWithUnitOpt().isPresent()) {
            num = node.getUnitNumberResolution().getNumber().get().intValue();
        } else {
            Log.debug("No UnitNumberResolution/UnitNumber present!", "ASTPort");
        }
        pas.setDimension(num);
        for (int i = 1; i <= num; ++i) {
            String nameWithArray = name + "[" + Integer.toString(i) + "]";
            EMAPortSymbol sym = new EMAPortSymbol(nameWithArray);
            sym.setNameDependsOn(pas.getNameDependsOn());
            Log.debug(nameWithArray, "nameWithArray");

            sym.setTypeReference(typeRef);
            sym.setDirection(node.isIncoming());

            symbolTableCreator.addToScopeAndLinkWithNode(sym, node);
        }
    }

    public static void portCreation(ASTPort node, EMAPortArraySymbol pas, String name,
                                    MCTypeReference<? extends MCTypeSymbol> typeRef,
                                    EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        if (node.getUnitNumberResolutionOpt().isPresent()) {
            portCreationIntLiteralPresent(node, pas, name, typeRef, symbolTableCreator);
        } else {
            // create EMAPortSymbol with same content as EMAPortArraySymbol
            createPort(node, name, node.isIncoming(), typeRef, pas, symbolTableCreator);
        }
    }

    public static void createPort(String name, boolean isIncoming,
                                  MCTypeReference<? extends MCTypeSymbol> typeRef,
                                  EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        EMAPortSymbol ps = new EMAPortSymbol(name);

        ps.setTypeReference(typeRef);
        ps.setDirection(isIncoming);

        symbolTableCreator.addToScope(ps);
    }

    public static void createPort(ASTPort node, String name, boolean isIncoming,
                                  MCTypeReference<? extends MCTypeSymbol> typeRef, EMAPortArraySymbol pas,
                                  EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        EMAPortSymbol ps = new EMAPortSymbol(name);
        ps.setNameDependsOn(pas.getNameDependsOn());
        ps.setTypeReference(typeRef);
        ps.setDirection(isIncoming);
        ps.setConfig(node.getAdaptableKeywordOpt().isPresent());

        symbolTableCreator.addToScopeAndLinkWithNode(ps, node);
    }

    public static String doPortResolution(ASTPort node, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        String name = null;
        if (node.getUnitNumberResolutionOpt().isPresent()) {
            ASTUnitNumberResolution unitNumberResolution = node.getUnitNumberResolution();
            name = unitNumberResolution.doResolution(symbolTableCreator.componentStack.
                    peek().getResolutionDeclarationSymbols());

        }
        return name;
    }


    public static List<String> getPortName(ASTQualifiedNameWithArrayAndStar portName,
                                           EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        List<String> names = new ArrayList<String>();

        List<String> compNameParts = getComponentNameParts(portName, symbolTableCreator);

        List<String> portNameParts;
        portNameParts = getPortNameParts(portName.getQualifiedNameWithArray(), symbolTableCreator);

        Log.debug("portName: " + portName.getQualifiedNameWithArray() + " " + compNameParts.size(), "CompNameParts");
        Log.debug("" + portNameParts.size(), "PortNameParts");
        for (String compNamePart : compNameParts) {
            for (String portNamePart : portNameParts) {
                String curName = compNamePart + portNamePart;
                names.add(curName);
            }
        }

        return names;
    }

    public static List<String> getComponentNameParts(ASTQualifiedNameWithArrayAndStar portNameStar,
                                                     EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        ASTQualifiedNameWithArray portName = portNameStar.getQualifiedNameWithArray();
		List<String> names = new ArrayList<String>();
        String name = "";
        if (portName.getCompNameOpt().isPresent()) {
            name += portName.getCompName();
            if (portName.getCompArrayOpt().isPresent()) {
                if (portName.getCompArray().getIntLiteralOpt().isPresent()) {
                    name += "[" + portName.getCompArray().getIntLiteral().getNumber().get().intValue()
                            + "]";
                    name += ".";
                    names.add(name);
                } else if (portName.getCompArray().getLowerboundOpt().isPresent()) {
                    names = getmnCompNameParts(name, portName);
                } else {
                    int size = countComponentArrayInstances(name, symbolTableCreator);
                    for (int i = 1; i <= size; ++i) {
                        String instanceName = name;
                        instanceName += "[" + i + "].";
                        names.add(instanceName);
                    }
                }
            } else {
                names.add(portName.getCompName() + ".");
            }
        } else {
            names.add("");
        }
        return names;
    }

    public static List<String> getmnCompNameParts(String name, ASTQualifiedNameWithArray portName) {
        List<String> names = new ArrayList<String>();
        int lower = portName.getCompArray().getLowerbound().getNumber().get().intValue();
        int upper = portName.getCompArray().getUpperbound().getNumber().get().intValue();
        for (int i = lower; i <= upper; ++i) {
            String instanceName = name;
            instanceName += "[" + i + "].";
            names.add(instanceName);
        }
        return names;
    }


    public static List<String> getPortNameParts(ASTQualifiedNameWithArray portName,
                                                EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        List<String> names = new ArrayList<String>();
        String name = "";
        // ignore for now
        if (portName.getCompNameOpt().isPresent())
            name += portName.getCompName() + ".";
        name = portName.getPortName();
        if (portName.getPortArrayOpt().isPresent()) {
            if (portName.getPortArray().getIntLiteralOpt().isPresent()) {
                name += "["
                        + portName.getPortArray().getIntLiteral().getNumber().get().intValue()
                        + "]";
                names.add(name);
            } else if (portName.getPortArray().getLowerboundOpt().isPresent()) {
                names = getmnPortNameParts(name, portName);
            } else {
                Log.debug(portName.toString(), "PortName:");
                int size = countPortArrayInstances(name, portName.getCompNameOpt().orElse(null),
                        portName.getCompArrayOpt().orElse(null), symbolTableCreator);

                Log.debug("Size" + size, "PortNameParts");
                for (int i = 1; i <= size; ++i) {
                    String instanceName = name;

                    instanceName += "[" + i + "]";

                    names.add(instanceName);
                }
            }
        } else {
            Log.debug("No PortArrayName was specified", "PortArray");
            names.add(portName.getPortName());
        }
        return names;
    }

    public static List<String> getmnPortNameParts(String name, ASTQualifiedNameWithArray portName) {
        List<String> names = new ArrayList<String>();
        int lower = portName.getPortArray().getLowerbound().getNumber().get().intValue();
        int upper = portName.getPortArray().getUpperbound().getNumber().get().intValue();
        for (int i = lower; i <= upper; ++i) {
            String instanceName = name;
            instanceName += "[" + i + "]";
            names.add(instanceName);
            Log.debug("Name:", "Added MNPortName");
        }
        return names;
    }


    public static String getNameArrayPart(ASTArrayAccess arrayPart) {
        String result = "";
        if (arrayPart.getIntLiteralOpt().isPresent())
            result += "[" + arrayPart.getIntLiteral().getNumber().get().intValue() + "]";
        // Not handled here change handling this case after refactoring
        /* else if (arrayPart.getLowerboundOpt().isPresent() && arrayPart.getUpperBoundOpt().isPresent()) {
         * } */
        return result;
    }

    public static int countPortArrayInstances(String portName, String compName, ASTArrayAccess arrayPart, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        MutableScope curScope = symbolTableCreator.currentScope().get();

        boolean present = true;
        int counter = 0;
        if (arrayPart != null) {
            compName += getNameArrayPart(arrayPart);
        }
        while (present) {
            Log.debug(compName, "ComponentName:");
            present = curScope.resolve(portName + "[" + (counter + 1) + "]", EMAPortSymbol.KIND).isPresent();
            if (present)
                ++counter;
            else {
                Log.debug(curScope.toString(), "CurScope:");
                Log.debug("Could not resolve " + portName + "[" + (counter + 1) + "]",
                        "countPortArrayInstances");
            }
        }
        if (counter == 0) {
            // TODO
            present = true;
            Log.debug("compInstanceName: " + compName, "Resolving");
            Log.debug(compName, "ComponentName");
            if (compName != null) {
                EMAComponentInstantiationSymbol symbol;
                symbol = curScope.<EMAComponentInstantiationSymbol>resolve(compName, EMAComponentInstantiationSymbol.KIND)
                        .get();
                for (EMAPortSymbol emaPortSymbol : symbol.getComponentType().getAllPorts()) {

                    Log.debug(emaPortSymbol.toString(), "PortInfo");
                    if (emaPortSymbol.getNameWithoutArrayBracketPart().startsWith(portName)) {
                        ++counter;
                    }
                }
            }
        }

        return counter;
    }

    public static int countComponentArrayInstances(String componentName, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        MutableScope curScope = symbolTableCreator.currentScope().get();
        boolean present = true;
        int counter = 0;
        Log.debug("" + componentName, "RESOLVING");
        while (present) {
            present = curScope
                    .resolve(componentName + "[" + (counter + 1) + "]", EMAComponentInstantiationSymbol.KIND)
                    .isPresent();
            if (present)
                ++counter;
        }
        return counter;
    }

    public static void starConnectorSetup(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.
                                                    ASTConnector node, EmbeddedMontiArcSymbolTableCreator
                                                    symbolTableCreator) {

        String sourceCmpName = node.getSource().getQualifiedNameWithArray().isPresentCompName() ? node.getSource().getQualifiedNameWithArray().getCompName() + "." : "";
        sourceCmpName += node.getSource().getQualifiedNameWithArray().getPortName();

        String targetCmpName = node.getTargets().getQualifiedNameWithArrayAndStar(0).getQualifiedNameWithArray().isPresentCompName() ?
                node.getTargets().getQualifiedNameWithArrayAndStar(0).getQualifiedNameWithArray().getCompName() + "." : "";
        targetCmpName += node.getTargets().getQualifiedNameWithArrayAndStar(0).getQualifiedNameWithArray().getPortName();
        EMAComponentSymbol sourceCmp, targetCmp;

        if(sourceCmpName.equals("this")) {
            sourceCmp = symbolTableCreator.componentStack.peek();
            targetCmp = sourceCmp.getSubComponent(targetCmpName).get().getComponentType().getReferencedSymbol();
            for(EMAPortSymbol from : sourceCmp.getIncomingPorts()) {
                for (EMAPortSymbol to : targetCmp.getIncomingPorts()) {
                    if (from.getName().equals(to.getName())) {
                        EMAConnectorSymbol connector = new EMAConnectorSymbol(to.getName());
                        connector.setSource(from.getName());
                        connector.setTarget(targetCmpName + "." + to.getName());
                        symbolTableCreator.addToScopeAndLinkWithNode(connector, node);
                    }
                }
            }
        } else if(targetCmpName.equals("this")) {
            targetCmp = symbolTableCreator.componentStack.peek();
            sourceCmp = targetCmp.getSubComponent(sourceCmpName).get().getComponentType().getReferencedSymbol();
            for(EMAPortSymbol from : sourceCmp.getOutgoingPorts()) {
                for (EMAPortSymbol to : targetCmp.getOutgoingPorts()) {
                    if (from.getName().equals(to.getName())) {
                        EMAConnectorSymbol connector = new EMAConnectorSymbol(to.getName());
                        connector.setSource(sourceCmpName + "." + from.getName());
                        connector.setTarget(to.getName());
                        symbolTableCreator.addToScopeAndLinkWithNode(connector, node);
                    }
                }
            }
        } else {

        }


    }

    public static void nonConstantPortSetup(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.
                                                    ASTConnector node, EmbeddedMontiArcSymbolTableCreator
                                                    symbolTableCreator) {

        List<String> sourceNames = getPortName(node.getSource(), symbolTableCreator);
        Log.info("" + sourceNames.size(), "SourcePorts");
        int counter = 0, targetnum = 0;
        for (ASTQualifiedNameWithArrayAndStar target : node.getTargets().getQualifiedNameWithArrayAndStarList()) {
            counter = 0;
            targetnum = 0;
            for (String sourceName : sourceNames) {
                List<String> targetNames = getPortName(target, symbolTableCreator);
                targetnum = targetNames.size();
                if (counter < targetnum) {
                    String targetName = targetNames.get(counter);
                    Log.info("" + targetName, "target");
                    Log.info("" + sourceName, "source");

                    EMAConnectorSymbol sym = new EMAConnectorSymbol(targetName);
                    sym.setSource(sourceName);
                    sym.setTarget(targetName);
                    Log.info(sym.getTarget(), "TARGETNAME SET TO");

                    symbolTableCreator.addToScopeAndLinkWithNode(sym, node);
                    ++counter;
                }
            }
            // TODO enable checking again if it is fixed
            /* if(counter!=targetnum) { Log.error("source port number "+ counter +" and target port num"+
             * targetnum+" don't match"); } */
        }
        if (node.getTargets().isHASH()) {
            String sourceName = getPortName(node.getSource(), symbolTableCreator).get(0);
            EMAConnectorSymbol sym = new EMAConnectorSymbol("#");
            sym.setSource(sourceName);
            sym.setTarget("#");
            symbolTableCreator.addToScopeAndLinkWithNode(sym, node);
        }
    }

    public static void constantPortSetup(
            de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTConnector node,
            EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        int counter = 0, targetnum = 0;
        EMAPortSymbol emaConstantPortSymbol = ConstantPortHelper.createConstantPortSymbol(node,
                symbolTableCreator);
        symbolTableCreator.addToScope(emaConstantPortSymbol);
        for (ASTQualifiedNameWithArrayAndStar target : node.getTargets().getQualifiedNameWithArrayAndStarList()) {
            counter = 0;
            targetnum = 0;
            List<String> targetNames = getPortName(target, symbolTableCreator);
            targetnum = targetNames.size();
            String targetName = targetNames.get(counter);
            Log.debug("" + targetName, "target");

            EMAConnectorSymbol sym = new EMAConnectorSymbol(targetName);
            sym.setConstantEMAPortSymbol(emaConstantPortSymbol);
            sym.setSource(emaConstantPortSymbol.getName());
            sym.setTarget(targetName);
            Log.debug(sym.getTarget(), "TARGETNAME SET TO");

            symbolTableCreator.addToScopeAndLinkWithNode(sym, node);
            ++counter;

        }
    }

    public static void doConnectorResolution(ASTConnector node, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        if (node.getUnitNumberResolutionOpt().isPresent()) {
            ASTUnitNumberResolution unitNumberResolution = node.getUnitNumberResolution();
            ASTNumberWithUnit toSet = null;
            if (unitNumberResolution.getNumberWithUnitOpt().isPresent()) {
                toSet = unitNumberResolution.getNumberWithUnit();
            } else if (unitNumberResolution.getNameOpt().isPresent()) {

                ResolutionDeclarationSymbol resDeclSym = symbolTableCreator.componentStack.peek()
                        .getResolutionDeclarationSymbol(unitNumberResolution.getNameOpt().get()).get();
                toSet = ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).getNumberWithUnit();
            }

            unitNumberResolution.setUnit(toSet.getUnit());
            unitNumberResolution.setNumber(toSet.getNumber().get());
        }

    }


    public static int handleSizeResolution(ASTSubComponent node, int index) {
        int counter = 0;
        if (node.getType() instanceof ASTSimpleReferenceType) {
            if (((ASTSimpleReferenceType) node.getType()).getTypeArgumentsOpt().isPresent()) {
                for (ASTTypeArgument typeArgument : ((ASTSimpleReferenceType) node.getType())
                        .getTypeArgumentsOpt().get().getTypeArgumentList()) {
                    if (typeArgument instanceof ASTUnitNumberTypeArgument) {
                        Log.debug("" + ((ASTUnitNumberTypeArgument) typeArgument).getNumberWithUnit().getNumber()
                                .get().intValue(), "New Resolution Value:");
                        if (counter == index)
                            return ((ASTUnitNumberTypeArgument) typeArgument).getNumberWithUnit().getNumber().get()
                                    .intValue();
                        ++counter;
                    }
                }
            }
        }
        return -1;
    }
}
