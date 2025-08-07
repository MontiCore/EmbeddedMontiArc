/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct._symboltable;

import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._ast.ASTStructFieldDefinition;
import de.monticore.lang.monticar.struct._cocos.StructCoCoChecker;
import de.monticore.lang.monticar.struct.coco.DefaultStructCoCoChecker;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.*;
import de.monticore.types.types._ast.ASTArrayType;
import de.monticore.types.types._ast.ASTComplexReferenceType;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTType;
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
        String packageQualifiedName = Names.getQualifiedName(node.getPackageList());
        List<ImportStatement> imports = node.getImportStatementList()
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
        super.endVisit(node);
        coCoChecker.checkAll(node);
    }

    @Override
    protected void initialize_StructFieldDefinition(StructFieldDefinitionSymbol structFieldDefinition, ASTStructFieldDefinition ast) {
        MCTypeReference<? extends MCTypeSymbol> type = getType(ast.getType(), currentScope().orElse(null));
        structFieldDefinition.setType(type);
    }

    private static String getTypeName(ASTElementType elementType) {
        return elementType.getName();
    }

    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTElementType elementType, Scope scope) {
        String name = getTypeName(elementType);
        return new CommonMCTypeReference<>(name, MCTypeSymbol.KIND, scope);
    }

    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTSimpleReferenceType astType, Scope scope) {
        if (astType.getTypeArgumentsOpt().isPresent()) {
            throw new UnsupportedOperationException("struct may not have type arguments");
        }
        String name = Names.getQualifiedName(astType.getNameList());
        return new CommonMCTypeReference<>(name, MCTypeSymbol.KIND, scope);
    }

    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTComplexReferenceType astType, Scope scope) {
        List<ASTSimpleReferenceType> srt = astType.getSimpleReferenceTypeList();
        if (srt.size() != 1) {
            throw new UnsupportedOperationException("nested structs are not allowed");
        }
        return getType(srt.get(0), scope);
    }

    private static MCTypeReference<? extends MCTypeSymbol> getType(ASTArrayType astType, Scope scope) {
        MCTypeReference<? extends MCTypeSymbol> type = getType(astType.getComponentType(), scope);
        type.setDimension(astType.getDimensions());
        return type;
    }

    public static MCTypeReference<? extends MCTypeSymbol> getType(ASTType astType, Scope scope) {
        MCTypeReference<? extends MCTypeSymbol> type = null;
        if (astType instanceof ASTElementType) {
            type = getType((ASTElementType) astType, scope);
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
        if (type == null)
            throw new UnsupportedOperationException("type " + astType + " is not supported");
        return type;
    }
}
