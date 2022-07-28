/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modularcnn;


import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._visitor.EmbeddedMontiArcVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._visitor.EmbeddedMontiArcDynamicVisitor;
import de.se_rwth.commons.logging.Log;


public class NetworkStructureScanner implements EmbeddedMontiArcVisitor {

    public NetworkStructureScanner() {

    }

    @Override
    public void visit(ASTNode node) {
        EmbeddedMontiArcVisitor.super.visit(node);
        Log.info("Hello","NSSV");
    }
}
