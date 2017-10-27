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
package de.monticore.lang.monticar.struct._symboltable;

import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._ast.ASTStructFieldDefinition;
import de.monticore.lang.monticar.struct._cocos.StructCoCoChecker;
import de.monticore.lang.monticar.struct.coco.DefaultStructCoCoChecker;
import de.monticore.lang.monticar.struct.model.type.StructFieldTypeInfo;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;

import java.util.Deque;
import java.util.List;
import java.util.stream.Collectors;

public class StructSymbolTableCreator extends StructSymbolTableCreatorTOP {
    private final StructCoCoChecker coCoChecker = DefaultStructCoCoChecker.create();

    public StructSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public StructSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    @Override
    public void visit(ASTStructCompilationUnit node) {
        coCoChecker.checkAll(node);
        String packageQualifiedName = Names.getQualifiedName(node.getPackage());
        List<ImportStatement> imports = node.getImportStatements()
                .stream()
                .map(imprt -> {
                    String qualifiedImport = Names.getQualifiedName(imprt.getImportList());
                    return new ImportStatement(qualifiedImport, imprt.isStar());
                })
                .collect(Collectors.toList());
        ArtifactScope artifactScope = new ArtifactScope(packageQualifiedName, imports);
        putOnStack(artifactScope);
    }

    @Override
    protected void initialize_StructFieldDefinition(StructFieldDefinitionSymbol structFieldDefinition, ASTStructFieldDefinition ast) {
        StructFieldTypeInfo typeInfo = StructFieldTypeInfo.tryRepresentASTType(ast.getType(), currentScope().orElse(null));
        structFieldDefinition.setTypeInfo(typeInfo);
    }
}
