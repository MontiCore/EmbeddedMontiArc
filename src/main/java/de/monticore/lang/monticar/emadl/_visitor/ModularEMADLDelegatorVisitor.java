/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl._visitor;

import java.util.Optional;

public class ModularEMADLDelegatorVisitor extends EMADLDelegatorVisitor implements EMADLInheritanceVisitor {

    private Optional<ModularNetworkVisitor> modularNetworkVisitor = Optional.empty();
    private ModularEMADLDelegatorVisitor realThis = this;

    public ModularEMADLDelegatorVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EMADLVisitor realThis) {
        super.setRealThis(realThis);
        this.realThis = (ModularEMADLDelegatorVisitor) realThis;

        if (this.modularNetworkVisitor.isPresent()){
            this.setModularNetworkVisitor(modularNetworkVisitor.get());
        }
    }

    public void setModularNetworkVisitor(ModularNetworkVisitor mnv) {
        this.modularNetworkVisitor = Optional.ofNullable(mnv);
        if (this.modularNetworkVisitor.isPresent()) {
            this.modularNetworkVisitor.get().setRealThis(getRealThis());
        }

        if (getRealThis() != this) {
            getRealThis().setModularNetworkVisitor(mnv);
        }
    }

}
