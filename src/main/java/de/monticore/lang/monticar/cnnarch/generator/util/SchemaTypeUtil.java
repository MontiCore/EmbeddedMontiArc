package de.monticore.lang.monticar.cnnarch.generator.util;

import schemalang._ast.ASTSchemaMember;
import schemalang._ast.ASTTypedDeclaration;
import schematypes.TypeIdentifier;
import schematypes._ast.ASTObjectType;
import schematypes._ast.ASTSchemaType;

public class SchemaTypeUtil {


    @SuppressWarnings("unused")
    public static String mapSchemaMemberToPythonType(ASTSchemaMember schemaMember) {
        //noinspection SwitchStatementWithTooFewBranches
        switch (schemaMember.getSchemaMemberType()) {
            case BASIC:
                return mapTypedDeclarationToPythonType(((ASTTypedDeclaration) schemaMember).getType());
            default:
                throw
                        new IllegalArgumentException("Schema type not handled");
        }
    }

    //TODO FMU further types ?
    private static String mapTypedDeclarationToPythonType(final ASTSchemaType schemaMemberType) {

        TypeIdentifier schemaMemberTypeIdentifier = schemaMemberType.getSchemaLangType();
        switch (schemaMemberTypeIdentifier) {
            case NATURAL_NUMBER_0:
            case NATURAL_NUMBER_1:
                return PythonType.INTEGER.getType();
            case BOOLEAN:
                return PythonType.BOOLEAN.getType();
            case COMPONENT:
            case STRING:
                return PythonType.STRING.getType();
            default:
                return "";
        }
    }

    @SuppressWarnings("unused")
    public boolean isPrimitiveSchemaMember(ASTSchemaMember astSchemaMember) {
        return astSchemaMember instanceof ASTTypedDeclaration && !(((ASTTypedDeclaration) astSchemaMember).getType() instanceof ASTObjectType);
    }
}
