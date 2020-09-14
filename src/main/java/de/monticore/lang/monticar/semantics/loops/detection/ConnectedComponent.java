/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;

import java.util.Collection;
import java.util.Map;
import java.util.Set;

public interface ConnectedComponent {
    public Collection<EMAVertex> getAllComponents();
    public EMAGraph getGraph();
    public LoopKind getKind();
    public void setKind(LoopKind kind);
    public Set<EMAPort> getInports();
    public void setInports(Set<EMAPort> inports);
    public Set<EMAPort> getOutports();
    public void setOutports(Set<EMAPort> outports);
    public Map<EMAPort, MathAssignmentExpressionSymbol> getPortStatements();
    public void addPortStatement(EMAPort port, MathAssignmentExpressionSymbol statement);
}
