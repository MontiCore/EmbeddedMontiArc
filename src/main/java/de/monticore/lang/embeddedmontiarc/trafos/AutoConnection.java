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
package de.monticore.lang.embeddedmontiarc.trafos;

import de.monticore.lang.embeddedmontiarc.EmbeddedMontiArcConstants;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.helper.AutoconnectMode;
import de.monticore.lang.embeddedmontiarc.helper.PortCompatibilityChecker;
import de.monticore.lang.monticar.common2._ast.ASTArrayAccess;
import de.monticore.lang.monticar.common2._ast.ASTQualifiedNameWithArray;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.numberunit._parser.NumberUnitParser;
import de.monticore.symboltable.MutableScope;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.io.IOException;
import java.util.*;
import java.util.Map.Entry;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

/**
 * Creates further connectors depending on the auto connect mode (type, port, off)
 *
 * @author ahaber, Robert Heim
 * @since 1.0.0 (14.11.2012)
 */
public class AutoConnection {

    private Stack<List<AutoconnectMode>> modeStack = new Stack<List<AutoconnectMode>>();

    private static ASTQualifiedNameWithArray parsePortName(String portName) {
        ASTQualifiedNameWithArray astQNWA = EmbeddedMontiArcNodeFactory.createASTQualifiedNameWithArray();
        ASTArrayAccess PortArray = EmbeddedMontiArcNodeFactory.createASTArrayAccess();
        ASTArrayAccess CompArray = EmbeddedMontiArcNodeFactory.createASTArrayAccess();
        ASTNumberWithUnit PortIntLiteral = EmbeddedMontiArcNodeFactory.createASTNumberWithUnit();
        ASTNumberWithUnit CompIntLiteral = EmbeddedMontiArcNodeFactory.createASTNumberWithUnit();

        String pattern = "([^\\[\\.]+)(\\[(\\d+)\\])?(\\.([^\\[\\.]+)(\\[(\\d+)\\])?)?";
        Pattern r = Pattern.compile(pattern);
        Matcher m = r.matcher(portName);
        if (m.group(4) == null) {
            astQNWA.setPortName(m.group(1));
            if (m.group(2) != null) {
                //PortIntLiteral.getIntLiteral().setSource(m.group(3));
                PortIntLiteral.setNumber(Rational.valueOf(m.group(3)).doubleValue());
                //TODO check if missing causes error PortArray.setIntLiteral(PortIntLiteral);
                astQNWA.setPortArray(PortArray);
            }
        } else {
            astQNWA.setCompName(m.group(2));
            if (m.group(2) != null) {
                //CompIntLiteral.getIntLiteral().setSource(m.group(3));
                CompIntLiteral.setNumber(Rational.valueOf(m.group(3)).doubleValue());
                //TODO check if missing causes error CompArray.setIntLiteral(CompIntLiteral);
                astQNWA.setCompArray(CompArray);
            }
            astQNWA.setPortName(m.group(5));
            if (m.group(6) != null) {
                //PortIntLiteral.getIntLiteral().setSource(m.group(7));
                NumberUnitParser parser = new NumberUnitParser();
                try {
                    Optional<ASTNumberWithUnit> portIntLitOpt = parser.parse_StringNumberWithUnit(m.group(7));
                    if (portIntLitOpt.isPresent())
                        PortIntLiteral = portIntLitOpt.get();
                    else
                        Log.error(String.format("Can not read number with unit: %s", m.group(7)));
                } catch (IOException e) {
                    Log.error(String.format("Can not read number with unit: %s", m.group(7), e));
                }
                //TODO check if missing causes error PortArray.setIntLiteral(PortIntLiteral);
                astQNWA.setPortArray(PortArray);
            }
        }
        return astQNWA;
    }

    public static void addConnectorToAST(ASTComponent node, EMAConnectorSymbol conEntry) {
        // create ast node
        ASTConnector astConnector = EmbeddedMontiArcNodeFactory.createASTConnector();

        //String pattern = "([^\\[\\.]+)(\\[(\\d+)\\])?(\\.([^\\[\\.]+)(\\[(\\d+)\\])?)?";
        ASTQualifiedNameWithArray source = EmbeddedMontiArcNodeFactory.createASTQualifiedNameWithArray();
    /*ASTArrayAccess srcPortArray = EmbeddedMontiArcNodeFactory.createASTArrayAccess();
    ASTArrayAccess srcCompArray = EmbeddedMontiArcNodeFactory.createASTArrayAccess();
    ASTIntLiteral srcPortIntLiteral = EmbeddedMontiArcNodeFactory.createASTIntLiteral();
    ASTIntLiteral srcCompIntLiteral = EmbeddedMontiArcNodeFactory.createASTIntLiteral();
    //source.setParts(Splitters.DOT.splitToList(conEntry.getSource()));*/
        String portName = conEntry.getSource();
        source = parsePortName(portName);

    /*Pattern r = Pattern.compile(pattern);
    Matcher m = r.matcher(portName);
    if (m.group(4) == null){
      source.setPortName(m.group(1));
      if (m.group(2) != null) {
        srcPortIntLiteral.setSource(m.group(3));
        srcPortArray.setIntLiteral(srcPortIntLiteral);
        source.setPortArray(srcPortArray);
      }
    }else{
      source.setCompName(m.group(2));
      if (m.group(2) != null){
        srcCompIntLiteral.setSource(m.group(3));
        srcCompArray.setIntLiteral(srcCompIntLiteral);
        source.setCompArray(srcCompArray);
      }

      source.setPortName(m.group(5));
      if (m.group(6) != null){
        srcPortIntLiteral.setSource(m.group(7));
        srcPortArray.setIntLiteral(srcPortIntLiteral);
        source.setPortArray(srcPortArray);
      }*/

        List<ASTQualifiedNameWithArray> targets = new ArrayList<>();
        ASTQualifiedNameWithArray target = EmbeddedMontiArcNodeFactory.createASTQualifiedNameWithArray();
        //target.setParts(Splitters.DOT.splitToList(conEntry.getTarget()));
        portName = conEntry.getTarget();
        target = parsePortName(portName);
        targets.add(target);

        astConnector.setSource(source);
        astConnector.setTargets(new ASTConnectorTargets(targets));

        Optional<ASTMontiArcAutoConnect> auto = resolveAutoconnect(node);
        if (auto.isPresent()) {
            astConnector.set_SourcePositionStart(auto.get().get_SourcePositionStart());
            astConnector.set_SourcePositionEnd(auto.get().get_SourcePositionEnd());
        }
        // add node to arc elements
        node.getBody().getElementList().add(astConnector);
    }

    /**
     * @param comp used component node.
     * @return searches the autoconnect statement in the given comp ast. returns empty, if it does not
     * exist.
     */
    public static Optional<ASTMontiArcAutoConnect> resolveAutoconnect(ASTComponent comp) {
        for (ASTElement element : comp.getBody().getElementList()) {
            if (element instanceof ASTMontiArcAutoConnect) {
                return Optional.of((ASTMontiArcAutoConnect) element);
            }
        }
        return Optional.empty();
    }

    public void transformAtStart(ASTComponent node, EMAComponentSymbol currentComp) {
        modeStack.push(new ArrayList<>());
    }

    public void transform(ASTMontiArcAutoConnect node, EMAComponentSymbol currentComp) {
        List<AutoconnectMode> modes = modeStack.peek();
        // add current mode
        if (node.isPort()) {
            modes.add(AutoconnectMode.AUTOCONNECT_PORT);
        } else if (node.isType()) {
            modes.add(AutoconnectMode.AUTOCONNECT_TYPE);
        } else if (node.isOff()) {
            modes.add(AutoconnectMode.OFF);
        }
    }

    public void transformAtEnd(ASTComponent node, EMAComponentSymbol currentComp) {
        List<AutoconnectMode> allModes = this.modeStack.peek();
        if (allModes.isEmpty()) {
            allModes.add(EmbeddedMontiArcConstants.DEFAULT_AUTO_CONNECT);
        }
        for (AutoconnectMode mode : allModes) {
            if (mode != AutoconnectMode.OFF && currentComp.getSubComponents().size() > 0) {
                createAutoconnections(currentComp, node, mode);
            }
        }
        this.modeStack.pop();
    }

    /**
     * Creates further connectors depending on the auto connect mode (type, port, off).
     *
     * @param currentComponent symbol table entry of the currently processed component
     * @param node             component node in the AST
     * @param mode             auto connection mode
     */
    private void createAutoconnections(EMAComponentSymbol currentComponent, ASTComponent node,
                                       AutoconnectMode mode) {
        Map<String, PorWithGenericBindings> unusedSenders = getUnusedSenders(
                currentComponent);
        Map<String, PorWithGenericBindings> unusedReceivers = getUnusedReceivers(
                currentComponent);
        for (Entry<String, PorWithGenericBindings> receiverEntry : unusedReceivers.entrySet()) {
            List<EMAConnectorSymbol> matches = new LinkedList<>();
            for (Entry<String, PorWithGenericBindings> senderEntry : unusedSenders.entrySet()) {
                handleSenderEntry(receiverEntry, senderEntry, matches, mode);
            }

            handleReceiverEntry(receiverEntry, currentComponent, matches, node);
        }
    }

    private void handleReceiverEntry(Entry<String, PorWithGenericBindings> receiverEntry, EMAComponentSymbol currentComponent, List<EMAConnectorSymbol> matches, ASTComponent node) {
        if (matches.size() == 1) {
            EMAConnectorSymbol created = matches.iterator().next();
            // add symbol to components scope
            ((MutableScope) currentComponent.getSpannedScope()).add(created);

            // add to ast
            addConnectorToAST(node, created);

            Log.info(node.get_SourcePositionStart() + " Created connector '" + created + "'.",
                    "AutoConnection");
        } else if (matches.size() > 1) {
            StringBuilder sb = new StringBuilder();
            sb.append("Duplicate autoconnection matches for the connector target '");
            sb.append(matches.iterator().next().getTarget());
            sb.append("' with the sources '");
            sb.append(matches.stream().map(c -> c.getSource()).collect(Collectors.joining("', '")));
            sb.append("'!");
            Log.warn(sb.toString());
        } else {
            Log.warn("Unable to autoconnect port '" + receiverEntry.getKey() + "'.");
        }
    }

    private void handleSenderEntry(Entry<String, PorWithGenericBindings> receiverEntry, Entry<String, PorWithGenericBindings> senderEntry, List<EMAConnectorSymbol> matches, AutoconnectMode mode) {
        String receiver = receiverEntry.getKey();
        int indexReceiver = receiver.indexOf('.');
        String sender = senderEntry.getKey();
        int indexSender = sender.indexOf('.');
        boolean matched = false;
        PorWithGenericBindings senderPort = senderEntry.getValue();
        PorWithGenericBindings receiverPort = receiverEntry.getValue();
        boolean portTypesMatch = PortCompatibilityChecker.doPortTypesMatch(senderPort.port,
                senderPort.formalTypeParameters, senderPort.typeArguments, receiverPort.port,
                receiverPort.formalTypeParameters, receiverPort.typeArguments);
        // sender from current component, receiver from a reference
        if (portTypesMatch) {
            if (indexSender == -1 && indexReceiver != -1) {
                String receiverPortName = receiver.substring(indexReceiver + 1);

                if (mode == AutoconnectMode.AUTOCONNECT_PORT && receiverPortName.equals(sender)) {
                    matched = true;
                } else if (mode == AutoconnectMode.AUTOCONNECT_TYPE) {
                    matched = true;
                }

            }
            // sender from a reference, receiver from current component
            else if (indexSender != -1 && indexReceiver == -1) {
                String senderPortName = sender.substring(indexSender + 1);

                if (mode == AutoconnectMode.AUTOCONNECT_PORT && senderPortName.equals(receiver)) {
                    matched = true;
                } else if (mode == AutoconnectMode.AUTOCONNECT_TYPE) {
                    matched = true;
                }

            }
            // both from referenced components
            else if (indexSender != -1 && indexReceiver != -1) {
                String senderPortName = sender.substring(indexSender + 1);
                String receiverPortName = receiver.substring(indexReceiver + 1);
                String senderRef = sender.substring(0, indexSender);
                String receiverRef = receiver.substring(0, indexReceiver);
                // check if sender and receiver are from different references
                if (!senderRef.equals(receiverRef)) {
                    if (mode == AutoconnectMode.AUTOCONNECT_PORT
                            && senderPortName.equals(receiverPortName)) {
                        matched = true;
                    } else if (mode == AutoconnectMode.AUTOCONNECT_TYPE) {
                        matched = true;
                    }

                }
            }
        }
        // create connector entry and add to matched
        if (matched) {
            EMAConnectorSymbol conEntry = EMAConnectorSymbol.builder()
                    .setSource(sender).setTarget(receiver).build();
            conEntry.setSource(sender);
            conEntry.setTarget(receiver);
            matches.add(conEntry);
        }
    }

    /**
     * Finds all incoming ports of a given component (and all referenced components) which are not
     * connected.
     *
     * @param currentComponent the component for which to find the unused ports
     * @return unused incoming ports of a given component represented in a map from port names to
     * types
     */
    private Map<String, PorWithGenericBindings> getUnusedReceivers(
            EMAComponentSymbol currentComponent) {
        // portname, porttypebinding
        Map<String, PorWithGenericBindings> unusedReceivers = new HashMap<>();
        // check outgoing ports, b/c they must receive data from within the component
        for (EMAPortSymbol receiver : currentComponent.getOutgoingPorts()) {
            if (!currentComponent.hasConnector(receiver.getName())) {
                unusedReceivers.put(receiver.getName(),
                        PorWithGenericBindings.create(receiver, currentComponent.getFormalTypeParameters(),
                                new ArrayList<>()));
            }
        }

        // check subcomponents incoming ports, b/c they must receive data to do their calculations
        for (EMAComponentInstantiationSymbol ref : currentComponent.getSubComponents()) {
            String name = ref.getName();
            EMAComponentSymbolReference refType = ref.getComponentType();
            for (EMAPortSymbol port : refType.getIncomingPorts()) {
                String portNameInConnector = name + "." + port.getName();
                if (!currentComponent.hasConnector(portNameInConnector)) {
                    // store the the type parameters' bindings of the referenced component for this reference
                    unusedReceivers.put(portNameInConnector,
                            PorWithGenericBindings.create(port,
                                    refType.getReferencedSymbol().getFormalTypeParameters(),
                                    refType.getActualTypeArguments().stream()
                                            .map(a -> (MCTypeReference<?>) a.getType())
                                            .collect(Collectors.toList())));
                }
            }
        }
        return unusedReceivers;
    }

    /**
     * Finds all outgoing ports of a given component (and all referenced components) which are not
     * connected.
     *
     * @param currentComponent the component for which to find the unused ports
     * @return unused outgoing ports of a given component represented in a map from port names to
     * types
     */
    private Map<String, PorWithGenericBindings> getUnusedSenders(
            EMAComponentSymbol currentComponent) {
        // portname, <port, formaltypeparameters for the subcomponent that the port is defined in, type
        // arguments for the subcomponent that the port is defined in>
        Map<String, PorWithGenericBindings> unusedSenders = new HashMap<>();
        // as senders could send to more then one receiver, all senders are added
        for (EMAPortSymbol sender : currentComponent.getIncomingPorts()) {
            if (!currentComponent.hasConnectors(sender.getName())) {
                unusedSenders.put(sender.getName(),
                        PorWithGenericBindings.create(sender, currentComponent.getFormalTypeParameters(),
                                new ArrayList<>()));
            }
        }

        // check subcomponents outputs as they send data to the current component
        for (EMAComponentInstantiationSymbol ref : currentComponent.getSubComponents()) {
            String name = ref.getName();
            EMAComponentSymbolReference refType = ref.getComponentType();
            for (EMAPortSymbol port : refType.getOutgoingPorts()) {
                String portNameInConnector = name + "." + port.getName();
                if (!currentComponent.hasConnectors(portNameInConnector)) {
                    // store the the type parameters' bindings of the referenced component for this reference
                    unusedSenders.put(portNameInConnector,
                            PorWithGenericBindings.create(port,
                                    refType.getReferencedSymbol().getFormalTypeParameters(),
                                    refType.getActualTypeArguments().stream()
                                            .map(a -> (MCTypeReference<?>) a.getType())
                                            .collect(Collectors.toList())));
                }
            }
        }
        return unusedSenders;
    }

    private static class PorWithGenericBindings {
        EMAPortSymbol port;

        List<MCTypeSymbol> formalTypeParameters;

        List<MCTypeReference<? extends MCTypeSymbol>> typeArguments;

        public PorWithGenericBindings(
                EMAPortSymbol port,
                List<MCTypeSymbol> formalTypeParameters,
                List<MCTypeReference<? extends MCTypeSymbol>> typeArguments) {
            super();
            this.port = port;
            this.formalTypeParameters = formalTypeParameters;
            this.typeArguments = typeArguments;
        }

        public static PorWithGenericBindings create(EMAPortSymbol port,
                                                    List<MCTypeSymbol> formalTypeParameters,
                                                    List<MCTypeReference<? extends MCTypeSymbol>> typeArguments) {
            return new PorWithGenericBindings(port, formalTypeParameters, typeArguments);
        }

    }
}
