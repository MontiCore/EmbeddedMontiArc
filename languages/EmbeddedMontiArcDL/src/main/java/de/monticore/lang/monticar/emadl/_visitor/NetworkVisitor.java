/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl._visitor;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;

public interface NetworkVisitor extends EMADLVisitor {

    default NetworkVisitor getRealThis() {
        return this;
    }

    default void setRealThis(NetworkVisitor mvn){
        throw new UnsupportedOperationException("setRealThis setter for ModularNetworkVisitor as ModularNetworkVisitor not implemented");
    }

    @Override
    default void visit(ASTNode node) {}

    @Override
    default void endVisit(ASTNode node) {}

    @Override
    default void visit(ASTEMACompilationUnit node){}

    @Override
    default void endVisit(ASTEMACompilationUnit node){}

    @Override
    default void visit(ASTEMADLNode node) {}

    @Override
    default void endVisit(ASTEMADLNode node) {}
}
