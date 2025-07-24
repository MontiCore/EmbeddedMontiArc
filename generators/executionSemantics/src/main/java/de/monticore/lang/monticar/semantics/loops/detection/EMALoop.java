/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.detection;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;

import java.util.Collection;

public interface EMALoop {
    public Collection<EMAComponentInstanceSymbol> getAllComponents();

    public EMAGraph getGraph();

    public boolean isArtificial();
}
