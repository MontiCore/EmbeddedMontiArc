/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl._visitor;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;

import java.util.Optional;

public class ModularEMADLDelegatorVisitor extends EMADLDelegatorVisitor implements EMADLInheritanceVisitor {

    private ModularEMADLDelegatorVisitor realThis = this;

    public ModularEMADLDelegatorVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(EMADLVisitor realThis) {
        super.setRealThis(realThis);
        if (this.realThis != realThis) {
            this.realThis = (ModularEMADLDelegatorVisitor) realThis;
            if (this.modularNetworkVisitor.isPresent()) {
                this.setModularNetworkVisitor(modularNetworkVisitor.get());
            }
        }
    }

    private Optional<NetworkVisitor> modularNetworkVisitor = Optional.empty();

    public Optional<NetworkVisitor> getModularNetworkVisitor() {
        return modularNetworkVisitor;
    }

    public void setModularNetworkVisitor(NetworkVisitor mvn) {
        this.modularNetworkVisitor = Optional.ofNullable(mvn);
        if (this.modularNetworkVisitor.isPresent()) {
            this.modularNetworkVisitor.get().setRealThis(getRealThis());
        }
        if (getRealThis() != this) {
            getRealThis().setModularNetworkVisitor(mvn);
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
        super.endVisit(node);
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().endVisit(node);
        }


    }

    @Override
    public void visit(ASTEMACompilationUnit node) {
        super.visit(node);
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().visit(node);
        }
    }


    @Override
    public void endVisit(ASTEMACompilationUnit node) {
        super.endVisit(node);
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().endVisit(node);
        }
    }

    @Override
    public void visit(ASTEMADLNode node){
        super.visit(node);
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().visit(node);
        }
    }

    @Override
    public void endVisit(ASTEMADLNode node){
        super.endVisit(node);
        if (getRealThis().getModularNetworkVisitor().isPresent()) {
            getRealThis().getModularNetworkVisitor().get().endVisit(node);
        }
    }

}
