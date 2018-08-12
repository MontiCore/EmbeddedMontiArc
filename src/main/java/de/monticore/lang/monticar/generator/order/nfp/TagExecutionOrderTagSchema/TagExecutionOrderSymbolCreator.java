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
package de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema;

import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;

/**
 * Created by ernst on 15.07.2016.
 */
public class TagExecutionOrderSymbolCreator implements TagSymbolCreator {

  public void create(ASTTaggingUnit unit, TaggingResolver gs) {
    if (unit.getQualifiedNameList().stream()
        .map(q -> q.toString())
        .filter(n -> n.endsWith("TagExecutionOrderTagSchema"))
        .count() == 0) {
      return; // the tagging model is not conform to the TagExecutionOrderTagSchema tagging schema
    }
    final String packageName = Joiners.DOT.join(unit.getPackageList());
    final String rootCmp = // if-else does not work b/cpp of final (required by streams)
        (unit.getTagBody().getTargetModelOpt().isPresent()) ?
            Joiners.DOT.join(packageName,
                unit.getTagBody().getTargetModelOpt().get()
                    .getQualifiedNameString()) :
            packageName;
  }

  @Override
  public void create(ASTTaggingUnit astTaggingUnit, Scope scope) {
    //TODO implement me, required by newer version
  }
}
