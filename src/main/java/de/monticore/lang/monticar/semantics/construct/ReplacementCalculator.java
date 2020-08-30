/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.EMAEdge;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.lang.monticar.semantics.resolve.ConstantsCalculator;

import java.io.IOException;
import java.io.StringReader;
import java.util.*;
import java.util.stream.Collectors;

public class ReplacementCalculator {

    private final String synthPath = "target/generated-components";
    private final String synthNamePostFix = "_synth";
    private final String synthPackagePreFix = "synth";

    private Set<ComponentReplacement> componentReplacements;
    private Set<ConnectorReplacement> connectorReplacements;
    private Set<PortReplacement> portReplacements;

    public void calculateReplacementsAndGenerateComponents
            (EMAComponentInstanceSymbol rootComponent, StrongConnectedComponent strongConnectedComponent, Set<EMAVertex> componentsToReplace,
             Map<String, String> solutionForPort) {

        componentReplacements = new HashSet<>();
        connectorReplacements = new HashSet<>();
        portReplacements = new HashSet<>();
        MathComponentGenerator generator = new MathComponentGenerator();

        for (EMAVertex vertexToReplace : componentsToReplace) {
            String parentComponent = vertexToReplace.getReferencedSymbol().getParent().get().getFullName();
            String type = vertexToReplace.getReferencedSymbol().getComponentType().getName() + synthNamePostFix;
            String packageName = synthPackagePreFix + "." + vertexToReplace.getReferencedSymbol().getPackageName();
            Map<String, String> inports = new HashMap<>();
            Map<String, String> outports = new HashMap<>();
            List<String> mathStatements = new LinkedList<>();

            for (EMAPort inport : vertexToReplace.getInports()) {
                inports.put(inport.getName(), inport.getReferencedPort().getTypeReference().getName());
            }
            for (EMAPort outport : vertexToReplace.getOutports()) {
                outports.put(outport.getName(), outport.getReferencedPort().getTypeReference().getName());
                mathStatements.add(outport.getName() + "=" + solutionForPort.get(outport.getFullName()));
            }

            // Analyze math Statements in order to redirect new input ports
            // Dependend constants should be output ports of some components or the input ports of the rootcomponent
            Set<String> dependendConstants = getDependedConstants(mathStatements, outports.keySet());
            Set<EMAPort> dependingPorts = dependendConstants.stream().map(
                    s -> strongConnectedComponent.getGraph().getPortMap().get(s)).collect(Collectors.toSet());

            for (EMAPort dependingPort : dependingPorts) {
                String nameOfInputPort = "";
                Optional<EMAPort> inport = getAlreadyConnectingPort(strongConnectedComponent, vertexToReplace, dependingPort);
                if (inport.isPresent()) {
                    nameOfInputPort = inport.get().getName();
                } else {
                    // Calculate redirections
                    ConnectSourceWithTargetPort connectSourceWithTargetPort = new ConnectSourceWithTargetPort(rootComponent, dependingPort, vertexToReplace);
                    Set<ConnectorReplacement> newConnectors = connectSourceWithTargetPort.getNewConnectors();
                    Set<PortReplacement> newPorts = connectSourceWithTargetPort.getNewPorts();

                    PortReplacement newInport = newPorts.stream().filter(p -> p.getComponent().equals(vertexToReplace.getFullName()))
                            .collect(Collectors.toList()).get(0);
                    inports.put(newInport.getName(), newInport.getType());
                    nameOfInputPort = newInport.getName();

                    newPorts.remove(newInport);
                    connectorReplacements.addAll(newConnectors);
                    portReplacements.addAll(newPorts);
                }

                // Replace the name of the depening port with the existing connecting / new connecting port
                ListIterator<String> statement = mathStatements.listIterator();
                while (statement.hasNext()) {
                    String s = statement.next();
                    statement.set(s.replace(dependingPort.getFullName(), nameOfInputPort));
                }
            }

            generator.generate(type, packageName, inports, outports, mathStatements, synthPath);

            ComponentReplacement replacement = new ComponentReplacement(parentComponent, vertexToReplace.getName(),
                    packageName, type, vertexToReplace.getName());
            componentReplacements.add(replacement);
        }
    }

    private Optional<EMAPort> getAlreadyConnectingPort(StrongConnectedComponent strongConnectedComponent,
                                                       EMAVertex emaVertex, EMAPort port) {
        EMAGraph graph = strongConnectedComponent.getGraph();
        List<EMAEdge> edgesWithSourcePort = graph.getEdgesWithSourcePort(port);
        for (EMAEdge emaEdge : edgesWithSourcePort) {
            if (emaEdge.getTargetVertex() == emaVertex) {
                return Optional.of(emaEdge.getTargetPort());
            }
        }
        return Optional.empty();
    }

    private Set<String> getDependedConstants(List<String> mathStatements, Set<String> variables) {
        Set<String> res = new HashSet<>();
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        for (String mathStatement : mathStatements) {
            ASTExpression expr = null;
            String parseName = mathStatement.replace(".", "_");
            try {
                expr = parser.parseExpression(new StringReader(parseName)).get();
            } catch (IOException e) {
                e.printStackTrace();
            }

            ConstantsCalculator constantsCalculator = new ConstantsCalculator(variables);
            expr.accept(constantsCalculator);
            res.addAll(constantsCalculator.getConstants().stream().map(
                    s -> s = s.replace("_", ".")).collect(Collectors.toSet()));
        }
        return res;
    }

    public Set<ComponentReplacement> getComponentReplacements() {
        return componentReplacements;
    }

    public Set<ConnectorReplacement> getConnectorReplacements() {
        return connectorReplacements;
    }

    public Set<PortReplacement> getPortReplacements() {
        return portReplacements;
    }
}
