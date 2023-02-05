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
    default void visit(ASTNode node) {
        //Log.info("MVN","VISIT_MVN");
    }


    @Override
    default void endVisit(ASTNode node) {
        //Log.info("MVN","END_VISIT_MVN");

    }

    @Override
    default void visit(ASTEMACompilationUnit node){
        //Log.info("MVN","VISIT_MVN_COMP");

    }

    @Override
    default void endVisit(ASTEMACompilationUnit node){
        //Log.info("MVN","END_VISIT_MVN_COMP");
    }

    @Override
    default void visit(ASTEMADLNode node) {
        //Log.info("MVN","VISIT_MVN");
    }

    @Override
    default void endVisit(ASTEMADLNode node) {
        //Log.info("MVN","END_VISIT_MVN");

    }
}
