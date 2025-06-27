/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class StronglyConnectedComponent implements EMALoop {
    private Set<EMAComponentInstanceSymbol> emaComponents;

    private Set<SimpleCycle> simpleCycles;
    private EMAGraph graph;
    private boolean artificial;

    public StronglyConnectedComponent(Set<EMAComponentInstanceSymbol> emaComponents,
                                      Set<List<EMAComponentInstanceSymbol>> simpleCycles,
                                      EMAGraph graph) {
        this.emaComponents = emaComponents;
        this.simpleCycles = new HashSet<>();
        for (List<EMAComponentInstanceSymbol> simpleCycle : simpleCycles) {
            this.simpleCycles.add(new SimpleCycle(graph, simpleCycle));
        }
        this.graph = graph;
    }

    @Override
    public Set<EMAComponentInstanceSymbol> getAllComponents() {
        return emaComponents;
    }

    @Override
    public EMAGraph getGraph() {
        return graph;
    }

    @Override
    public boolean isArtificial() {
        return artificial;
    }

    public void setArtificial(boolean artificial) {
        this.artificial = artificial;
        for (SimpleCycle simpleCycle : simpleCycles)
            simpleCycle.setArtificial(artificial);
    }

    public Set<SimpleCycle> getSimpleCycles() {
        return simpleCycles;
    }
}
