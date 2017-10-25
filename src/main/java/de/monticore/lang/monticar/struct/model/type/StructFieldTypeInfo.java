package de.monticore.lang.monticar.struct.model.type;

import de.monticore.lang.monticar.struct._symboltable.StructSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTComplexArrayType;
import de.monticore.lang.monticar.types2._ast.ASTComplexReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTElementArrayType;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTPrimitiveArrayType;
import de.monticore.lang.monticar.types2._ast.ASTPrimitiveType;
import de.monticore.lang.monticar.types2._ast.ASTSimpleReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTType;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Names;

import java.util.List;

/**
 * Marker interface which unites possible type descriptors of a structure field.
 */
public interface StructFieldTypeInfo {

    /**
     * Tries to represent input {@link ASTType} with
     * one of the representatives of the {@link StructFieldTypeInfo}.
     *
     * @param node           AST node describing type
     * @param enclosingScope Enclosing scope of the input AST node.
     * @return one of the representatives of the {@link StructFieldTypeInfo}
     * (i.e. implementors of the interface) or {@code null}
     * if the type cannot be handled.
     */
    static StructFieldTypeInfo tryRepresentASTType(ASTType node, Scope enclosingScope) {
        if (node instanceof ASTPrimitiveType) {
            return tryRepresentASTPrimitiveType((ASTPrimitiveType) node);
        } else if (node instanceof ASTElementType) {
            return tryRepresentASTElementType((ASTElementType) node);
        } else if (node instanceof ASTSimpleReferenceType) {
            return tryRepresentASTSimpleReferenceType((ASTSimpleReferenceType) node, enclosingScope);
        } else if (node instanceof ASTPrimitiveArrayType) {
            return tryRepresentASTPrimitiveArrayType((ASTPrimitiveArrayType) node);
        } else if (node instanceof ASTComplexArrayType) {
            return tryRepresentASTComplexArrayType((ASTComplexArrayType) node, enclosingScope);
        } else if (node instanceof ASTElementArrayType) {
            return tryRepresentASTElementArrayType((ASTElementArrayType) node);
        }
        return null;
    }

    static ScalarStructFieldType tryRepresentASTPrimitiveType(ASTPrimitiveType type) {
        if (type.isBoolean()) {
            return ScalarStructFieldType.BOOLEAN;
        } else if (type.isByte()) {
            return ScalarStructFieldType.BYTE;
        } else if (type.isShort()) {
            return ScalarStructFieldType.SHORT;
        } else if (type.isInt()) {
            return ScalarStructFieldType.INT;
        } else if (type.isLong()) {
            return ScalarStructFieldType.LONG;
        } else if (type.isChar()) {
            return ScalarStructFieldType.CHAR;
        } else if (type.isFloat()) {
            return ScalarStructFieldType.FLOAT;
        } else if (type.isDouble()) {
            return ScalarStructFieldType.DOUBLE;
        } else {
            return null;
        }
    }

    static ScalarStructFieldType tryRepresentASTElementType(ASTElementType type) {
        if (type.isIsWholeNumberNumber()) {
            return ScalarStructFieldType.INTEGRAL;
        } else if (type.isIsRational()) {
            return ScalarStructFieldType.RATIONAL;
        } else if (type.isIsComplex()) {
            return ScalarStructFieldType.COMPLEX;
        } else if (type.isIsBoolean()) {
            return ScalarStructFieldType.BOOL;
        } else {
            return null;
        }
    }

    static StructReferenceFieldType tryRepresentASTSimpleReferenceType(ASTSimpleReferenceType type, Scope enclosingScope) {
        StructSymbolReference reference = new StructSymbolReference(
                Names.getQualifiedName(type.getNames()),
                enclosingScope
        );
        StructReferenceFieldType referenceFieldType = new StructReferenceFieldType();
        referenceFieldType.setReference(reference);
        return referenceFieldType;
    }

    static VectorStructFieldType tryRepresentASTPrimitiveArrayType(ASTPrimitiveArrayType type) {
        ASTPrimitiveType astPrimitiveType = (ASTPrimitiveType) type.getComponentType();
        ScalarStructFieldType typeOfElements = tryRepresentASTPrimitiveType(astPrimitiveType);
        return VectorStructFieldType.of(typeOfElements, type.getDimensions());
    }

    static VectorStructFieldType tryRepresentASTComplexArrayType(ASTComplexArrayType type, Scope enclosingScope) {
        ASTComplexReferenceType astComplexReferenceType = (ASTComplexReferenceType) type.getComponentType();
        List<ASTSimpleReferenceType> simpleReferenceTypes = astComplexReferenceType.getSimpleReferenceTypes();
        if (simpleReferenceTypes.size() != 1) {
            return null;
        }
        StructReferenceFieldType typeOfElements = tryRepresentASTSimpleReferenceType(
                simpleReferenceTypes.get(0),
                enclosingScope
        );
        return VectorStructFieldType.of(typeOfElements, type.getDimensions());
    }

    static VectorStructFieldType tryRepresentASTElementArrayType(ASTElementArrayType type) {
        ScalarStructFieldType typeOfElements = tryRepresentASTElementType(
                (ASTElementType) type.getComponentType()
        );
        return VectorStructFieldType.of(typeOfElements, type.getDimensions());
    }
}
