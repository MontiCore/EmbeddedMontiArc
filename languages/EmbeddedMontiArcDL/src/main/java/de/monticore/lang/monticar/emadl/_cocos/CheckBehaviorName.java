/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._cocos;

import de.monticore.lang.monticar.emadl._ast.ASTBehaviorEmbedding;
import de.monticore.lang.monticar.emadl._ast.ASTBehaviorName;
import de.se_rwth.commons.logging.Log;

public class CheckBehaviorName implements EMADLASTBehaviorNameCoCo, EMADLASTBehaviorEmbeddingCoCo {

    private ASTBehaviorEmbedding behaviorEmbedding = null;
    private ASTBehaviorName behaviorName = null;

    @Override
    public void check(ASTBehaviorEmbedding node) {
        behaviorEmbedding = node;
        if (behaviorName != null){
            checkBehavior();
        }
    }

    @Override
    public void check(ASTBehaviorName node) {
        behaviorName = node;
        if (behaviorEmbedding != null){
            checkBehavior();
        }
    }

    private void checkBehavior() {
        if (behaviorName.getName().equals("Math")){
            if (behaviorEmbedding.getStatementList() == null){
                Log.error("Implementation Name 'Math' is incorrect."
                        , behaviorName.get_SourcePositionStart());
            }
        }
        else {
            if (!behaviorEmbedding.isPresentArchitecture()){
                Log.error("Implementation Name 'Math' is incorrect."
                        , behaviorName.get_SourcePositionStart());
            }
        }
    }
}
