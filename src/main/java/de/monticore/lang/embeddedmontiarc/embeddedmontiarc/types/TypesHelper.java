/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types;

import com.google.common.base.Preconditions;
import com.google.common.base.Strings;
import de.monticore.types.types._ast.*;
import de.se_rwth.commons.Names;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Created by dennisqiao on 3/8/17.
 */
public class TypesHelper {
    public static final String OPTIONAL = "Optional";

    public TypesHelper() {
    }

    public static boolean isOptional(ASTType type) {
        return isGenericTypeWithOneTypeArgument(type, "Optional");
    }

    public static boolean isPrimitive(ASTType type) {
        return type instanceof ASTPrimitiveType;
    }

    public static ASTTypeArgument getReferenceTypeFromOptional(ASTType type) {
        Preconditions.checkArgument(isOptional(type));
        return (ASTTypeArgument) ((ASTTypeArguments) ((ASTSimpleReferenceType) type).getTypeArgumentsOpt().get()).getTypeArgumentList().get(0);
    }

    public static ASTSimpleReferenceType getSimpleReferenceTypeFromOptional(ASTType type) {
        Preconditions.checkArgument(isOptional(type));
        ASTTypeArgument refType = getReferenceTypeFromOptional(type);
        if (refType instanceof ASTWildcardType && ((ASTWildcardType) refType).getUpperBoundOpt().isPresent()) {
            refType = (ASTTypeArgument) ((ASTWildcardType) refType).getUpperBound();
        }

        Preconditions.checkState(refType instanceof ASTSimpleReferenceType);
        return (ASTSimpleReferenceType) refType;
    }

    public static String getReferenceNameFromOptional(ASTType type) {
        Preconditions.checkArgument(isOptional(type));
        ASTTypeArgument reference = (ASTTypeArgument) ((ASTTypeArguments) ((ASTSimpleReferenceType) type).getTypeArgumentsOpt().get()).getTypeArgumentList().get(0);
        if (reference instanceof ASTWildcardType && ((ASTWildcardType) reference).getUpperBoundOpt().isPresent()) {
            reference = (ASTTypeArgument) ((ASTWildcardType) reference).getUpperBound();
        }

        Preconditions.checkArgument(reference instanceof ASTSimpleReferenceType);
        List names = ((ASTSimpleReferenceType) reference).getNameList();
        return names.isEmpty() ? "" : (String) names.get(names.size() - 1);
    }

    public static String getQualifiedReferenceNameFromOptional(ASTType type) {
        Preconditions.checkArgument(isOptional(type));
        ASTTypeArgument reference = (ASTTypeArgument) ((ASTTypeArguments) ((ASTSimpleReferenceType) type).getTypeArgumentsOpt().get()).getTypeArgumentList().get(0);
        if (reference instanceof ASTWildcardType && ((ASTWildcardType) reference).getUpperBoundOpt().isPresent()) {
            reference = (ASTTypeArgument) ((ASTWildcardType) reference).getUpperBound();
        }

        Preconditions.checkArgument(reference instanceof ASTSimpleReferenceType);
        List names = ((ASTSimpleReferenceType) reference).getNameList();
        return names.isEmpty() ? "" : Names.getQualifiedName(names);
    }

    public static boolean isGenericTypeWithOneTypeArgument(ASTType type, String simpleRefTypeName) {
        if (!(type instanceof ASTSimpleReferenceType)) {
            return false;
        } else {
            ASTSimpleReferenceType simpleRefType = (ASTSimpleReferenceType) type;
            return Names.getQualifiedName(simpleRefType.getNameList()).equals(simpleRefTypeName) && simpleRefType.getTypeArgumentsOpt().isPresent() && ((ASTTypeArguments) simpleRefType.getTypeArgumentsOpt().get()).getTypeArgumentList().size() == 1;
        }
    }

    public static int getArrayDimensionIfArrayOrZero(ASTType astType) {
        return astType instanceof ASTArrayType ? ((ASTArrayType) astType).getDimensions() : 0;
    }

    public static Optional<ASTSimpleReferenceType> getFirstTypeArgumentOfGenericType(ASTType type, String simpleRefTypeName) {
        if (!isGenericTypeWithOneTypeArgument(type, simpleRefTypeName)) {
            return Optional.empty();
        } else {
            ASTSimpleReferenceType simpleRefType = (ASTSimpleReferenceType) type;
            ASTTypeArgument typeArgument = (ASTTypeArgument) ((ASTTypeArguments) simpleRefType.getTypeArgumentsOpt().get()).getTypeArgumentList().get(0);
            return !(typeArgument instanceof ASTSimpleReferenceType) ? Optional.empty() : Optional.of((ASTSimpleReferenceType) typeArgument);
        }
    }

    public static Optional<ASTSimpleReferenceType> getFirstTypeArgumentOfOptional(ASTType type) {
        return getFirstTypeArgumentOfGenericType(type, "Optional");
    }

    public static String getSimpleName(ASTSimpleReferenceType simpleType) {
        String name = "";
        List qualifiedName = simpleType.getNameList();
        if (qualifiedName != null && !qualifiedName.isEmpty()) {
            name = (String) qualifiedName.get(qualifiedName.size() - 1);
        }

        return name;
    }

    public static List<String> createListFromDotSeparatedString(String s) {
        return Arrays.asList(s.split("\\."));
    }

    public static String printType(ASTType type) {
        if (isOptional(type)) {
            ASTTypeArgument ref = getReferenceTypeFromOptional(type);
            return printType(ref);
        } else {
            return TypesPrinter.printType(type);
        }
    }

    public static boolean isNullable(ASTType type) {
        return !isPrimitive(type);
    }

    public static String printType(ASTTypeArgument type) {
        return type instanceof ASTWildcardType ? TypesPrinter.printWildcardType((ASTWildcardType) type) : printType((ASTType) type);
    }

    public static String printSimpleRefType(ASTType type) {
        return isOptional(type) ? printType((ASTType) getSimpleReferenceTypeFromOptional(type)) : TypesPrinter.printType(type);
    }

    public static int getPrimitiveType(String typeName) {
        if (Strings.isNullOrEmpty(typeName)) {
            return -1;
        } else {
            byte var2 = -1;
            updateVar2(var2, typeName);
            int result = -1;
            switch (var2) {
                case 0:
                    result = 1;
                    break;
                case 1:
                    result = 7;
                    break;
                case 2:
                    result = 2;
                    break;
                case 3:
                    result = 6;
                    break;
                case 4:
                    result = 8;
                    break;
                case 5:
                    result = 4;
                    break;
                case 6:
                    result = 3;
                    break;
                case 7:
                    result = 5;
                    break;
                default:
                    result = -1;
            }
            return result;
        }
    }

    private static int updateVar2(int var2, String typeName) {
        switch (typeName.hashCode()) {
            case -1325958191:
                if (typeName.equals("double")) var2 = 4;
                break;
            case 104431:
                if (typeName.equals("int")) var2 = 5;
                break;
            case 3039496:
                if (typeName.equals("byte")) var2 = 2;
                break;
            case 3052374:
                if (typeName.equals("char")) var2 = 3;
                break;
            case 3327612:
                if (typeName.equals("long")) var2 = 7;
                break;
            case 64711720:
                if (typeName.equals("boolean")) var2 = 0;
                break;
            case 97526364:
                if (typeName.equals("float")) var2 = 1;
                break;
            case 109413500:
                if (typeName.equals("short")) var2 = 6;
        }
        return var2;
    }
}
