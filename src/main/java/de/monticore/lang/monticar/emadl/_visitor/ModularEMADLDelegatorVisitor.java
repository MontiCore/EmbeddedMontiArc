/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl._visitor;

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.emadl.modularcnn.ModularCNNSymbolTableCreator;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class ModularEMADLDelegatorVisitor extends EMADLDelegatorVisitor implements EMADLInheritanceVisitor {

    private ModularEMADLDelegatorVisitor realThis = this;

    public ModularEMADLDelegatorVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EMADLVisitor realThis) {
        if (this.realThis != realThis){
            super.setRealThis(realThis);
            this.realThis = (ModularEMADLDelegatorVisitor) realThis;
            if (this.modularNetworkVisitor.isPresent()){
                Log.info("Trying to set MVN","MEDV_SetRealThis");
                this.setModularNetworkVisitor(modularNetworkVisitor.get());
            }
        }



    }

    private Optional<ModularNetworkVisitor> modularNetworkVisitor = Optional.empty();

    public Optional<ModularNetworkVisitor> getModularNetworkVisitor(){
        return modularNetworkVisitor;
    }

    public void setModularNetworkVisitor(ModularNetworkVisitor mvn) {
        this.modularNetworkVisitor = Optional.ofNullable(mvn);
        if (this.modularNetworkVisitor.isPresent()) {
            Log.info("Trying to set MVN","MEDV_setModularNetworkVisitor_1");
            this.modularNetworkVisitor.get().setRealThis(getRealThis());
        }
        Log.info("Trying to set MVN","MEDV_setModularNetworkVisitor_2");
        if (getRealThis() != this) {
            getRealThis().setModularNetworkVisitor(mvn);
        }
    }

    //@Override
    public void handle(ASTNode node) {
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().handle(node);
        }
    }

    //@Override
    public void traverse(ASTNode node) {
        //super.traverse(node)
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().traverse(node);
        }
    }

    @Override
    public void visit(ASTNode node) {
        super.visit(node);
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().visit(node);
        }
    }

    @Override
    public void endVisit(ASTNode node) {
        super.visit(node);
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().endVisit(node);
        }
    }



}
