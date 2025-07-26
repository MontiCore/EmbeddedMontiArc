/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import java.util.List;

public class EMAPortVertex extends EMAVertex {
    public EMAPortVertex(EMAComponentInstanceSymbol referencedSymbol, String name, String fullName, List<EMAPort> inports, List<EMAPort> outports) {
        super(referencedSymbol, name, fullName, inports, outports);
    }
}
