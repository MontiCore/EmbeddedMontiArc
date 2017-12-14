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
package de.monticore.lang.monticar.enumlang._symboltable;

import de.monticore.lang.monticar.enumlang._ast.ASTEnumLangCompilationUnit;
import de.monticore.lang.monticar.enumlang._cocos.EnumLangCoCoChecker;
import de.monticore.lang.monticar.enumlang.coco.DefaultEnumCoCoChecker;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;

import java.util.Collections;
import java.util.Deque;

public class EnumLangSymbolTableCreator extends EnumLangSymbolTableCreatorTOP {

    private final EnumLangCoCoChecker coCoChecker = DefaultEnumCoCoChecker.create();

    public EnumLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public EnumLangSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    @Override
    public void visit(ASTEnumLangCompilationUnit node) {
        coCoChecker.checkAll(node);
        String packageQualifiedName = Names.getQualifiedName(node.getPackage());
        ArtifactScope artifactScope = new ArtifactScope(packageQualifiedName, Collections.emptyList());
        putOnStack(artifactScope);
    }
}
