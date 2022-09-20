/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl._visitor;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._visitor.EmbeddedMontiArcVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._visitor.EmbeddedMontiArcDynamicVisitor;
import de.monticore.lang.mathopt._visitor.MathOptVisitor;
import de.monticore.lang.monticar.emadl._ast.ASTEMADLNode;
import de.se_rwth.commons.logging.Log;

public interface ModularNetworkVisitor extends EMADLVisitor {

    default ModularNetworkVisitor getRealThis() {
        return this;
    }

    default void setRealThis(ModularNetworkVisitor mvn){
        throw new UnsupportedOperationException("setRealThis setter for ModularNetworkVisitor as ModularNetworkVisitor not implemented");
    }

    /*
    default void setRealThis(EMADLVisitor v){
        throw new UnsupportedOperationException("setRealThis setter for ModularNetworkVisitor as EMADLVisitor not implemented");
    }*/

    //@Override
    default void traverse(ASTNode node){
        //Log.info("MVN","TRAVERSE_MVN");
    }

    //@Override
    default void handle(ASTNode node) {
        //Log.info("MVN","HANDLE_MVN");
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
