/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.monticar.semantics.Options;
import de.monticore.lang.monticar.semantics.loops.detection.ConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.EMAEdge;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.lang.monticar.semantics.resolve.ConstantsCalculator;
import org.apache.commons.lang3.StringUtils;

import java.io.IOException;
import java.io.StringReader;
import java.util.*;
import java.util.stream.Collectors;

public class ReplacementCalculator {

    private Replacement replacement;

    public ReplacementCalculator(Replacement replacement) {
        this.replacement = replacement;
    }

    public void calculateReplacementsAndGenerateComponents
            (EMAComponentInstanceSymbol rootComponent, ConnectedComponent strongConnectedComponent, Set<EMAVertex> componentsToReplace,
             Map<String, String> solutionForPort) {

        MathComponentGenerator generator = new MathComponentGenerator();

        for (EMAVertex vertexToReplace : componentsToReplace) {
            String parentComponent = vertexToReplace.getReferencedSymbol().getParent().get().getFullName();
            String type = StringUtils.capitalize(vertexToReplace.getName());
            String packageName = Options.synthPackagePreFix + "." + parentComponent;
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
            Set<EMAPort> dependingPorts = dependendConstants.stream().filter(s -> !isTime(s)).map(
                    s -> strongConnectedComponent.getGraph().getPortMap().get(s)).collect(Collectors.toSet());

            for (EMAPort dependingPort : dependingPorts) {
                String nameOfInputPort = "";
                Optional<EMAPort> inport = getAlreadyConnectingPort(strongConnectedComponent, vertexToReplace, dependingPort);
                if (inport.isPresent()) {
                    nameOfInputPort = inport.get().getName();
                } else {
                    // Calculate redirections
                    ConnectSourceWithTargetPort connectSourceWithTargetPort = new ConnectSourceWithTargetPort(replacement, rootComponent);
                    connectSourceWithTargetPort.addConnection(dependingPort, vertexToReplace);

                    PortReplacement newInport = replacement.getPortReplacements()
                            .stream()
                            .filter(p -> p.getComponent().equals(vertexToReplace.getFullName()))
                            .collect(Collectors.toList()).get(0);
                    inports.put(newInport.getName(), newInport.getType());
                    nameOfInputPort = newInport.getName();

                    replacement.remove(newInport);
                }

                // Replace the name of the depening port with the existing connecting / new connecting port
                ListIterator<String> statement = mathStatements.listIterator();
                while (statement.hasNext()) {
                    String s = statement.next();
                    statement.set(s.replace(dependingPort.getFullName(), nameOfInputPort));
                }
            }

            generator.generate(type, packageName, inports, outports, mathStatements, Options.synthPath);

            ComponentReplacement componentReplacement = new ComponentReplacement(parentComponent, vertexToReplace.getName(),
                    packageName, type, vertexToReplace.getName());

            replacement.add(componentReplacement);
        }
    }

    private boolean isTime(String s) {
        return "t".equals(s);
    }

    private Optional<EMAPort> getAlreadyConnectingPort(ConnectedComponent strongConnectedComponent,
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
}
