/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types;

import de.monticore.lang.monticar.printtype._ast.ASTPrintType;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.monticore.types.types._ast.*;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.Iterator;
import java.util.List;

/**
 */
public class TypesPrinterImpl {


    protected String doPrintType(ASTType type) {
        return type instanceof ASTArrayType ? this.doPrintArrayType((ASTArrayType) type) : (type instanceof ASTPrimitiveType ? this.doPrintPrimitiveType((ASTPrimitiveType) type) : (type instanceof ASTReferenceType ? this.doPrintReferenceType((ASTReferenceType) type) : ""));
    }

    protected String doPrintReferenceType(ASTReferenceType type) {
        return type instanceof ASTSimpleReferenceType ? this.doPrintSimpleReferenceType((ASTSimpleReferenceType) type) : (type instanceof ASTComplexReferenceType ? this.doPrintComplexReferenceType((ASTComplexReferenceType) type) : "");
    }

    protected String doPrintReturnType(ASTReturnType type) {
        return type instanceof ASTType ? this.doPrintType((ASTType) type) : (type instanceof ASTVoidType ? this.doPrintVoidType((ASTVoidType) type) : "");
    }

    protected String doPrintTypeArgument(ASTTypeArgument type) {
        return type instanceof ASTWildcardType ? this.doPrintWildcardType((ASTWildcardType) type) : (type instanceof ASTType ? this.doPrintType((ASTType) type) : "");
    }

    protected String doPrintTypeWithoutTypeArguments(ASTType type) {
        return type instanceof ASTArrayType ? this.doPrintArrayType((ASTArrayType) type) : (type instanceof ASTPrimitiveType ? this.doPrintPrimitiveType((ASTPrimitiveType) type) : (type instanceof ASTReferenceType ? this.doPrintReferenceTypeWithoutTypeArguments((ASTReferenceType) type) : ""));
    }

    protected String doPrintTypeWithoutTypeArgumentsAndDimension(ASTType type) {
        return type instanceof ASTArrayType ? this.doPrintTypeWithoutTypeArgumentsAndDimension(((ASTArrayType) type).getComponentType()) : (type instanceof ASTPrimitiveType ? this.doPrintPrimitiveType((ASTPrimitiveType) type) : (type instanceof ASTReferenceType ? this.doPrintTypeWithoutTypeArguments((ASTReferenceType) type) : resolveNewType(type)));
    }

    protected String resolveNewType(ASTType type) {
        if (type instanceof ASTPrintType) {
            return ((ASTPrintType) type).printType();
        } else if (type instanceof ASTElementType) {
            ASTElementType t = (ASTElementType) type;
            return t.getName();
        }

        Log.info(type.toString(), "Type:");
        Log.error("Type can not be handled!");
        return "";
    }


    protected String doPrintTypeParameters(ASTTypeParameters2 params) {
        return params != null && params.getTypeVariableDeclaration2List() != null && !params.getTypeVariableDeclaration2List().isEmpty() ? "<" + this.doPrintTypeVariableDeclarationList(params.getTypeVariableDeclaration2List()) + ">" : "";
    }


    protected String doPrintTypeVariableDeclarationList(List<ASTTypeVariableDeclaration2> decl) {
        StringBuilder ret = new StringBuilder();
        if (decl != null) {
            String sep = "";

            for (Iterator var4 = decl.iterator(); var4.hasNext(); sep = ", ") {
                ASTTypeVariableDeclaration2 d = (ASTTypeVariableDeclaration2) var4.next();
                ret.append(sep + this.doPrintTypeVariableDeclaration(d));
            }
        }

        return ret.toString();
    }

    protected String doPrintTypeVariableDeclaration(ASTTypeVariableDeclaration2 decl) {
        StringBuilder ret = new StringBuilder();
        if (decl != null) {
            ret.append(decl.getNamingResolution().getName());
            if (decl.getUpperBoundsList() != null && !decl.getUpperBoundsList().isEmpty()) {
                String sep = " extends ";

                for (Iterator var4 = decl.getUpperBoundsList().iterator(); var4.hasNext(); sep = " & ") {
                    ASTType type = (ASTType) var4.next();
                    ret.append(sep + this.doPrintType(type));
                }
            }
        }

        return ret.toString();
    }

    protected String doPrintVoidType(ASTVoidType type) {
        return type != null ? "void" : "";
    }

    protected String doPrintPrimitiveType(ASTPrimitiveType type) {
        return type == null ? "" : (type.getPrimitive() == 1 ? "boolean" : (type.getPrimitive() == 2 ? "byte" : (type.getPrimitive() == 6 ? "char" : (type.getPrimitive() == 3 ? "short" : (type.getPrimitive() == 4 ? "int" : (type.getPrimitive() == 7 ? "float" : (type.getPrimitive() == 5 ? "long" : (type.getPrimitive() == 8 ? "double" : ""))))))));
    }

    protected String doPrintArrayType(ASTArrayType type) {
        if (type == null) {
            return "";
        } else {
            StringBuilder dimension = new StringBuilder();
            dimension.append(this.doPrintType(type.getComponentType()));

            for (int i = 0; i < type.getDimensions(); ++i) {
                dimension.append("[]");
            }

            return dimension.toString();
        }
    }

    protected String doPrintReferenceTypeList(List<ASTReferenceType> type) {
        StringBuilder ret = new StringBuilder();
        if (type != null) {
            String sep = "";

            for (Iterator var4 = type.iterator(); var4.hasNext(); sep = ", ") {
                ASTReferenceType refType = (ASTReferenceType) var4.next();
                ret.append(sep + this.doPrintReferenceType(refType));
            }
        }

        return ret.toString();
    }

    protected String doPrintSimpleReferenceType(ASTSimpleReferenceType type) {
        return type != null ? (type.getTypeArgumentsOpt().isPresent() ? Names.getQualifiedName(type.getNameList()) + this.doPrintTypeArguments((ASTTypeArguments) type.getTypeArgumentsOpt().get()) : Names.getQualifiedName(type.getNameList())) : "";
    }

    protected String doPrintComplexReferenceType(ASTComplexReferenceType type) {
        String ret = "";
        return type != null && type.getSimpleReferenceTypeList() != null ? this.doPrintSimpleReferenceTypeList(type.getSimpleReferenceTypeList()) : ret;
    }

    protected String doPrintSimpleReferenceTypeList(List<ASTSimpleReferenceType> argList) {
        StringBuilder ret = new StringBuilder();
        if (argList != null) {
            String sep = "";

            for (Iterator var4 = argList.iterator(); var4.hasNext(); sep = ". ") {
                ASTSimpleReferenceType arg = (ASTSimpleReferenceType) var4.next();
                ret.append(sep + this.doPrintSimpleReferenceType(arg));
            }
        }

        return ret.toString();
    }

    protected String doPrintTypeArguments(ASTTypeArguments args) {
        return args != null && args.getTypeArgumentList() != null && !args.getTypeArgumentList().isEmpty() ? "<" + this.doPrintTypeArgumentList(args.getTypeArgumentList()) + ">" : "";
    }

    protected String doPrintTypeArgumentList(List<ASTTypeArgument> argList) {
        StringBuilder ret = new StringBuilder();
        if (argList != null) {
            String sep = "";

            for (Iterator var4 = argList.iterator(); var4.hasNext(); sep = ", ") {
                ASTTypeArgument arg = (ASTTypeArgument) var4.next();
                ret.append(sep + this.doPrintTypeArgument(arg));
            }
        }

        return ret.toString();
    }

    protected String doPrintWildcardType(ASTWildcardType type) {
        StringBuilder ret = new StringBuilder();
        if (type != null) {
            ret.append("?");
            if (type.getUpperBoundOpt().isPresent()) {
                ret.append(" extends " + this.doPrintType((ASTType) type.getUpperBound()));
            } else if (type.getLowerBoundOpt().isPresent()) {
                ret.append(" super " + this.doPrintType((ASTType) type.getLowerBound()));
            }
        }

        return ret.toString();
    }

    protected String doPrintReferenceTypeWithoutTypeArguments(ASTReferenceType type) {
        return type instanceof ASTSimpleReferenceType ? this.doPrintSimpleReferenceTypeWithoutTypeArguments((ASTSimpleReferenceType) type) : (type instanceof ASTComplexReferenceType ? this.doPrintComplexReferenceTypeWithoutTypeArguments((ASTComplexReferenceType) type) : "");
    }

    protected String doPrintSimpleReferenceTypeWithoutTypeArguments(ASTSimpleReferenceType type) {
        return type != null ? Names.getQualifiedName(type.getNameList()) : "";
    }

    protected String doPrintComplexReferenceTypeWithoutTypeArguments(ASTComplexReferenceType type) {
        return type != null && type.getSimpleReferenceTypeList() != null ? this.doPrintSimpleReferenceTypeListWithoutTypeArguments(type.getSimpleReferenceTypeList()) : "";
    }

    protected String doPrintSimpleReferenceTypeListWithoutTypeArguments(List<ASTSimpleReferenceType> argList) {
        StringBuilder ret = new StringBuilder();
        if (argList != null) {
            String sep = "";

            for (Iterator var4 = argList.iterator(); var4.hasNext(); sep = ". ") {
                ASTSimpleReferenceType arg = (ASTSimpleReferenceType) var4.next();
                ret.append(sep + this.doPrintSimpleReferenceTypeWithoutTypeArguments(arg));
            }
        }

        return ret.toString();
    }

}
