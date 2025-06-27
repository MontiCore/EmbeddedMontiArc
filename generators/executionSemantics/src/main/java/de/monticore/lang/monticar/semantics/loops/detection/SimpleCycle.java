/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;

import java.util.List;

public class SimpleCycle implements EMALoop {
    private EMAGraph graph;
    private List<EMAComponentInstanceSymbol> allComponents;
    private boolean artificial;

    public SimpleCycle(EMAGraph graph, List<EMAComponentInstanceSymbol> allComponents) {
        this.graph = graph;
        this.allComponents = allComponents;
    }

    @Override
    public List<EMAComponentInstanceSymbol> getAllComponents() {
        return allComponents;
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
    }
}
