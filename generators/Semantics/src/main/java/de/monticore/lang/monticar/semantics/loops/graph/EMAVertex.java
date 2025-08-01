package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;

import java.util.LinkedList;
import java.util.List;

public class EMAVertex {
    private String name = "";
    private String fullName = "";
    private EMAComponentInstanceSymbol referencedSymbol = null;
    private List<EMAPort> inports = new LinkedList<>();
    private List<EMAPort> outports  = new LinkedList<>();

    public EMAVertex(EMAComponentInstanceSymbol referencedSymbol, String name, String fullName,
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

    public static EMAVertex create(EMAComponentInstanceSymbol component) {
        String name = component.getName();
        String fullName = component.getFullName();
        List<EMAPort> inports = new LinkedList<>();
        List<EMAPort> outports = new LinkedList<>();
        for (EMAPortInstanceSymbol inport: component.getIncomingPortInstances()) {
            inports.add(new EMAPort(inport, inport.getFullName(), inport.getName()));
        }
        for (EMAPortInstanceSymbol outport: component.getOutgoingPortInstances()) {
            outports.add(new EMAPort(outport, outport.getFullName(), outport.getName()));
        }
        return new EMAVertex(component, name, fullName, inports, outports);
    }

    public String getName() {
        return name;
    }

    public String getFullName() {
        return fullName;
    }

    public EMAComponentInstanceSymbol getReferencedSymbol() {
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
