/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.graph;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;

import java.util.Objects;
import java.util.Optional;

public class EMAAtomicConnectorInstance extends EMADynamicConnectorInstanceSymbol {
    private EMAPortInstanceSymbol source;
    private EMAPortInstanceSymbol target;

    public EMAAtomicConnectorInstance(EMAPortInstanceSymbol source, EMAPortInstanceSymbol target) {
        super(target.getFullName());
        this.source = source;
        this.target = target;
        if (source.isConstant())
            setConstantEMAPortSymbol(source);
        else if (target.isConstant())
            setConstantEMAPortSymbol(target);
    }

    public EMAPortInstanceSymbol getSourcePort() {
        return source;
    }

    public EMAComponentInstanceSymbol getSourceComponent() {
        return getSourcePort().getComponentInstance();
    }

    public EMAPortInstanceSymbol getTargetPort() {
        return target;
    }

    public EMAComponentInstanceSymbol getTargetComponent() {
        return getTargetPort().getComponentInstance();
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof EMAAtomicConnectorInstance)) return false;
        if (!source.equals(((EMAAtomicConnectorInstance) obj).getSourcePort())) return false;
        if (!target.equals(((EMAAtomicConnectorInstance) obj).getTargetPort())) return false;
        return true;
    }


    @Override
    public EMAComponentInstanceSymbol getComponentInstance() {
        EMAComponentInstanceSymbol outerMostParent = getSourceComponent();

        while (outerMostParent.isVirtual() && outerMostParent.getParent().isPresent())
            outerMostParent = outerMostParent.getParent().get();

        return outerMostParent;
    }

    @Override
    public Optional<String> getSourceComponentName() {
        return Optional.ofNullable(getSourceComponent().getFullName());
    }

    @Override
    public String getSourcePortName() {
        return getSourcePort().getFullName();
    }

    @Override
    public Optional<String> getTargetComponentName() {
        return Optional.ofNullable(getTargetComponent().getFullName());
    }

    @Override
    public String getTargetPortName() {
        return getTargetPort().getFullName();
    }

    @Override
    public String getSource() {
        return getSourcePortName();
    }

    @Override
    public String getTarget() {
        return getTargetPortName();
    }

    @Override
    public int hashCode() {
        return Objects.hash(getSource(), getTarget());
    }
}
