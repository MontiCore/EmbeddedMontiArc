/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl._visitor;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._visitor.EmbeddedMontiArcVisitor;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._visitor.EmbeddedMontiArcDynamicVisitor;
import de.se_rwth.commons.logging.Log;

public class ModularNetworkVisitor implements EmbeddedMontiArcVisitor {

    private ModularNetworkVisitor realThis = this;


    public void setRealThis(ModularNetworkVisitor rt) {
       this.realThis = rt;
    }

    public void setRealThis(EmbeddedMontiArcVisitor rt){
        //this.realThis = (ModularNetworkVisitor) rt;
    }

    public ModularNetworkVisitor getRealThis() {
        return this;
    }

    public ModularNetworkVisitor(){
        Log.info("MNV IN DA HOUSE","INIT_MNV");
    }

    public void handle(ASTNode node) {
        Log.info("MNV","HANDLE_MNV");
    }

    public void visit(ASTNode node) {
        Log.info("MNV","VISIT_MNV");
    }

    public void endVisit(ASTNode node){
        Log.info("MNV","END_MNV");
    }


}
