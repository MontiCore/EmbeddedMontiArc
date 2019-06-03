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
package de.monticore.lang.monticar.cnntrain._cocos;

import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Sets;
import de.monticore.lang.monticar.cnntrain._ast.*;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class CheckEntryRepetition implements CNNTrainASTEntryCoCo {
    private final static Set<Class<? extends ASTEntry>> REPEATABLE_ENTRIES = ImmutableSet
        .<Class<? extends ASTEntry>>builder()
            .add(ASTOptimizerParamEntry.class)
            .build();



    private Set<String> entryNameSet = new HashSet<>();

    @Override
    public void check(ASTEntry node) {
        if (!isRepeatable(node)) {
            String parameterPrefix = "";

            if (node instanceof ASTGreedyEpsilonEntry) {
                parameterPrefix = "greedy_";
            }
            if (entryNameSet.contains(parameterPrefix + node.getName())) {
                Log.error("0" + ErrorCodes.ENTRY_REPETITION_CODE + " The parameter '" + node.getName() + "' has multiple values. " +
                                "Multiple assignments of the same parameter are not allowed",
                        node.get_SourcePositionStart());
            } else {
                entryNameSet.add(parameterPrefix + node.getName());
            }
        }
    }

    private boolean isRepeatable(final ASTEntry node) {
        return REPEATABLE_ENTRIES.stream().anyMatch(i -> i.isInstance(node));
    }

}
