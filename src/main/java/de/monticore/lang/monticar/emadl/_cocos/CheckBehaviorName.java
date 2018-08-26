/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
        if (behaviorName.getNameOpt().get().equals("Math")){
            if (!behaviorEmbedding.isPresentMathStatements()){
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
