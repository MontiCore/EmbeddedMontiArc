/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.math._ast.ASTMathAssignmentStatement;
import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;

import java.util.*;

public class StrongConnectedComponent implements ConnectedComponent {
    private Set<EMAVertex> emaComponents;

    private Set<SimpleCycle> simpleCycles;
    private EMAGraph graph;
    private LoopKind kind = LoopKind.Default;
    private Set<EMAPort> inports = new HashSet<>();
    private Set<EMAPort> outports = new HashSet<>();
    private Map<EMAPort, ASTMathAssignmentStatement> portStatements = new HashMap();

    public StrongConnectedComponent(Set<EMAVertex> emaComponents, Set<List<EMAVertex>> simpleCycles, EMAGraph graph) {
        this.emaComponents = emaComponents;
        this.simpleCycles = new HashSet<>();
        for (List<EMAVertex> simpleCycle : simpleCycles) {
            this.simpleCycles.add(new SimpleCycle(graph, simpleCycle));
        }
        this.graph = graph;
    }

    @Override
    public LoopKind getKind() {
        return kind;
    }

    @Override
    public void setKind(LoopKind kind) {
        this.kind = kind;
    }

    @Override
    public Set<EMAVertex> getAllComponents() {
        return emaComponents;
    }

    @Override
    public EMAGraph getGraph() {
        return graph;
    }

    public Set<SimpleCycle> getSimpleCycles() {
        return simpleCycles;
    }

    @Override
    public Set<EMAPort> getInports() {
        return inports;
    }

    @Override
    public void setInports(Set<EMAPort> inports) {
        this.inports = inports;
    }

    @Override
    public Set<EMAPort> getOutports() {
        return outports;
    }

    @Override
    public void setOutports(Set<EMAPort> outports) {
        this.outports = outports;
    }

    @Override
    public Map<EMAPort, ASTMathAssignmentStatement> getPortStatements() {
        return portStatements;
    }

    @Override
    public void addPortStatement(EMAPort port, ASTMathAssignmentStatement statement) {
        portStatements.put(port, statement);
        for (SimpleCycle simpleCycle : simpleCycles) {
            simpleCycle.addPortStatement(port, statement);
        }
    }
}
