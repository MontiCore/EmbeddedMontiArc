/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types;

import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.monticore.types.types._ast.*;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * Created by dennisqiao on 3/8/17.
 */
public class TypesPrinter {
    private static TypesPrinterImpl instance;

    private TypesPrinter() {
    }

    private static TypesPrinterImpl getInstance() {
        if (instance == null) {
            instance = new TypesPrinterImpl();
        }

        return instance;
    }

    public static String printReferenceType(ASTReferenceType type) {
        return getInstance().doPrintReferenceType(type);
    }

    public static String printType(ASTType type) {
        return getInstance().doPrintType(type);
    }

    public static String printReturnType(ASTReturnType type) {
        return getInstance().doPrintReturnType(type);
    }

    public static String printTypeArgument(ASTTypeArgument type) {
        return getInstance().doPrintTypeArgument(type);
    }

    public static String printTypeWithoutTypeArgumentsAndDimension(ASTType type) {
        return getInstance().doPrintTypeWithoutTypeArgumentsAndDimension(type);
    }

    public static String printTypeWithoutTypeArguments(ASTType type) {
        return getInstance().doPrintTypeWithoutTypeArguments(type);
    }

    public static String printTypeParameters(ASTTypeParameters2 params) {
        return getInstance().doPrintTypeParameters(params);
    }

    public static String printTypeVariableDeclarationList(List<ASTTypeVariableDeclaration2> decl) {
        return getInstance().doPrintTypeVariableDeclarationList(decl);
    }


    public static String printTypeVariableDeclaration(ASTTypeVariableDeclaration2 decl) {
        return getInstance().doPrintTypeVariableDeclaration(decl);
    }

    public static String printVoidType(ASTVoidType type) {
        return getInstance().doPrintVoidType(type);
    }

    public static String printPrimitiveType(ASTPrimitiveType type) {
        return getInstance().doPrintPrimitiveType(type);
    }

    public static String printArrayType(ASTArrayType type) {
        return getInstance().doPrintArrayType(type);
    }

    public static String printReferenceTypeList(List<ASTReferenceType> type) {
        return getInstance().doPrintReferenceTypeList(type);
    }

    public static String printComplexReferenceType(ASTComplexReferenceType type) {
        return getInstance().doPrintComplexReferenceType(type);
    }


    public static String printSimpleReferenceType(ASTSimpleReferenceType type) {
        return getInstance().doPrintSimpleReferenceType(type);
    }


    public static String printSimpleReferenceTypeList(List<ASTSimpleReferenceType> type) {
        return getInstance().doPrintSimpleReferenceTypeList(type);
    }

    public static String printTypeArguments(ASTTypeArguments args) {
        return getInstance().doPrintTypeArguments(args);
    }

    public static String printTypeArgumentList(List<ASTTypeArgument> argList) {
        return getInstance().doPrintTypeArgumentList(argList);
    }

    public static String printWildcardType(ASTWildcardType type) {
        return getInstance().doPrintWildcardType(type);
    }


    /**
     * Removes Optional from target name
     * so something like name[Optional[1/1]]
     * becomes name[1]
     *
     * @param targetName
     * @return
     */
    public static String fixTargetName(String targetName) {
        if (targetName.contains("[Optional[")) {
            int firstClosedBracket = targetName.indexOf("]");
            int secondClosedBracket = targetName.indexOf("]", firstClosedBracket + 1);
            if (secondClosedBracket != -1) {
                Log.info(targetName, "targetName:");
                int index = targetName.indexOf("[Optional[");
                index += "Optional[".length();
                int endIndex = targetName.indexOf("/", index);
                if (endIndex >= 0) {
                    String bracketNumber = targetName.substring(index, endIndex);
                    int endOfInnerBracket = targetName.indexOf("]", endIndex + 1);
                    int endOfSecondBracket = targetName.indexOf("]", endOfInnerBracket + 1);
                    return targetName.substring(0, index) + bracketNumber + "]" + targetName.substring(endOfSecondBracket + 1, targetName.length());
                }
            } else {
                return targetName.replaceAll("Optional\\[", "");
            }
        }
        return targetName;
    }

    public static String FirstLowerCase(String name) {
        String newName;
        if (name.length() > 1) {
            newName = Character.toLowerCase(name.charAt(0)) + name.substring(1);
        } else {
            newName = Character.toLowerCase(name.charAt(0)) + "";
        }

        return newName;
    }
}
