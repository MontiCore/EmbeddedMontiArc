package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.math._ast.ASTStatement;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class EMAVertex {
    private String name = "";
    private String fullName = "";
    private EMAComponentInstantiationSymbol referencedSymbol = null;
    private List<EMAPort> inports = new LinkedList<>();
    private List<EMAPort> outports  = new LinkedList<>();

    public EMAVertex (String name, String fullName) {
        this.name = name;
        this.fullName = fullName;
    }

    public EMAVertex(EMAComponentInstantiationSymbol referencedSymbol, String name, String fullName,
                     List<EMAPort> inports, List<EMAPort> outports) {
        this.name = name;
        this.fullName = fullName;
        this.referencedSymbol = referencedSymbol;
        this.inports = inports;
        this.outports = outports;
        for (EMAPort inport : inports) {
            inport.setEmaVertex(this);
        }
        for (EMAPort outport : outports) {
            outport.setEmaVertex(this);
        }
    }

    public static EMAVertex create(EMAComponentInstantiationSymbol symbol, String parentFullName) {
        String name = symbol.getName();
        String fullName = parentFullName + "." + name;
        List<EMAPort> inports = new LinkedList<>();
        List<EMAPort> outports = new LinkedList<>();
        for (EMAPortSymbol inport: symbol.getComponentType().getReferencedSymbol().getIncomingPorts()) {
            inports.add(new EMAPort(inport.getName(), fullName + "." + inport.getName(), inport));
        }
        for (EMAPortSymbol outport: symbol.getComponentType().getReferencedSymbol().getOutgoingPorts()) {
            outports.add(new EMAPort(outport.getName(), fullName + "." + outport.getName(), outport));
        }
        return new EMAVertex(symbol, name, fullName, inports, outports);
    }

    public String getName() {
        return name;
    }

    public String getFullName() {
        return fullName;
    }

    public EMAComponentInstantiationSymbol getReferencedSymbol() {
        return referencedSymbol;
    }

    public List<EMAPort> getInports() {
        return inports;
    }

    public List<EMAPort> getOutports() {
        return outports;
    }

    public String toString() {
        return fullName;
    }
}
