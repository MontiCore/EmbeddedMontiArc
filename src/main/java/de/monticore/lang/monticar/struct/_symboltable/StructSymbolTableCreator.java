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
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTArrayType;
import de.monticore.lang.monticar.types2._ast.ASTComplexReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTSimpleReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Scope;
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
    public void endVisit(ASTStructCompilationUnit node) {
        coCoChecker.checkAll(node);
    }

    @Override
    protected void initialize_StructFieldDefinition(StructFieldDefinitionSymbol structFieldDefinition, ASTStructFieldDefinition ast) {
        MCTypeReference<? extends MCTypeSymbol> type = getType(ast.getType(), currentScope().orElse(null));
        structFieldDefinition.setType(type);
    }

    private static String getTypeName(ASTElementType elementType){
        String name=null;
        if (elementType.isIsBoolean()) {
            name = "B";
        }
        if (elementType.isIsRational()) {
            name = "Q";
        }
        if (elementType.isIsComplex()) {
            name = "C";
        }
        if (elementType.isIsWholeNumberNumber()) {
            name = "Z";
        }  
        if (name == null) {
            throw new UnsupportedOperationException("ElementType " + elementType + " is not supported");
        }
        return name;
    }
    
    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTElementType elementType, Scope scope){
        String name = getTypeName(elementType);
        return new CommonMCTypeReference<>(name, MCTypeSymbol.KIND, scope);
    }
    
    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTSimpleReferenceType astType, Scope scope){
        if (astType.typeArgumentsIsPresent()) {
            throw new UnsupportedOperationException("struct may not have type arguments");
        }
        String name = Names.getQualifiedName(astType.getNames());
        return new CommonMCTypeReference<>(name, MCTypeSymbol.KIND, scope);
    }
    
    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTComplexReferenceType astType, Scope scope){
        List<ASTSimpleReferenceType> srt = astType.getSimpleReferenceTypes();
            if (srt.size() != 1) {
                throw new UnsupportedOperationException("nested structs are not allowed");
            }
            return getType(srt.get(0), scope);
    }
    
    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTArrayType astType, Scope scope){
        MCTypeReference<? extends MCTypeSymbol> type = getType(astType.getComponentType(), scope);
        type.setDimension(astType.getDimensions());
        return type;
    }
    
    public static MCTypeReference<? extends MCTypeSymbol> getType(ASTType astType, Scope scope) {
        MCTypeReference<? extends MCTypeSymbol> type = null;
        if (astType instanceof ASTElementType) {
            type = getType((ASTElementType) astType,scope);
        }
        if (astType instanceof ASTSimpleReferenceType) {
            type = getType((ASTSimpleReferenceType) astType, scope);    
        }
        if (astType instanceof ASTComplexReferenceType) {
            type = getType((ASTComplexReferenceType) astType, scope);
        }
        if (astType instanceof ASTArrayType) {
            type = getType((ASTArrayType) astType, scope);
        }
        if(type == null)
            throw new UnsupportedOperationException("type " + astType + " is not supported");
        return type;
    }
}
