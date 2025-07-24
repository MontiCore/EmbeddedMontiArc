/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;

import java.util.*;

public class SimpleCycle implements ConnectedComponent {
    private EMAGraph graph;
    private List<EMAVertex> allComponents;
    private LoopKind kind = LoopKind.Default;
    private Set<EMAPort> inports = new HashSet<>();
    private Set<EMAPort> outports = new HashSet<>();
    private Map<EMAPort, MathAssignmentExpressionSymbol> portStatements = new HashMap<>();

    public SimpleCycle(EMAGraph graph, List<EMAVertex> allComponents) {
        this.graph = graph;
        this.allComponents = allComponents;
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
    public List<EMAVertex> getAllComponents() {
        return allComponents;
    }

    @Override
    public EMAGraph getGraph() {
        return graph;
    }


    @Override
    public Map<EMAPort, MathAssignmentExpressionSymbol> getPortStatements() {
        return portStatements;
    }

    @Override
    public void addPortStatement(EMAPort port, MathAssignmentExpressionSymbol statement) {
        portStatements.put(port, statement);
    }
}
