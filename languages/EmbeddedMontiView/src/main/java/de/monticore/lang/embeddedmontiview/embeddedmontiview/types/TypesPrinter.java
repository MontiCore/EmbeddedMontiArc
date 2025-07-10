/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview.types;

import de.monticore.lang.monticar.resolution._ast.ASTTypeArgument;
import de.monticore.lang.monticar.types2._ast.*;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.Iterator;
import java.util.List;

/**
 * Created by dennisqiao on 3/8/17.
 */
public class TypesPrinter {
  private static TypesPrinter instance;

  private TypesPrinter() {
  }

  private static TypesPrinter getInstance() {
    if (instance == null) {
      instance = new TypesPrinter();
    }

    return instance;
  }

  public static String printType(ASTType type) {
    return getInstance().doPrintType(type);
  }

  protected String doPrintType(ASTType type) {
    return type instanceof ASTArrayType ? this.doPrintArrayType((ASTArrayType) type) : (type instanceof ASTPrimitiveType ? this.doPrintPrimitiveType((ASTPrimitiveType) type) : (type instanceof ASTReferenceType ? this.doPrintReferenceType((ASTReferenceType) type) : ""));
  }

  public static String printReferenceType(ASTReferenceType type) {
    return getInstance().doPrintReferenceType(type);
  }

  protected String doPrintReferenceType(ASTReferenceType type) {
    return type instanceof ASTSimpleReferenceType ? this.doPrintSimpleReferenceType((ASTSimpleReferenceType) type) : (type instanceof ASTComplexReferenceType ? this.doPrintComplexReferenceType((ASTComplexReferenceType) type) : "");
  }

  public static String printReturnType(ASTReturnType type) {
    return getInstance().doPrintReturnType(type);
  }

  protected String doPrintReturnType(ASTReturnType type) {
    return type instanceof ASTType ? this.doPrintType((ASTType) type) : (type instanceof ASTVoidType ? this.doPrintVoidType((ASTVoidType) type) : "");
  }

  public static String printTypeArgument(ASTTypeArgument type) {
    return getInstance().doPrintTypeArgument(type);
  }

  protected String doPrintTypeArgument(ASTTypeArgument type) {
    return type instanceof ASTWildcardType ? this.doPrintWildcardType((ASTWildcardType) type) : (type instanceof ASTType ? this.doPrintType((ASTType) type) : "");
  }

  public static String printTypeWithoutTypeArguments(ASTType type) {
    return getInstance().doPrintTypeWithoutTypeArguments(type);
  }

  protected String doPrintTypeWithoutTypeArguments(ASTType type) {
    return type instanceof ASTArrayType ? this.doPrintArrayType((ASTArrayType) type) : (type instanceof ASTPrimitiveType ? this.doPrintPrimitiveType((ASTPrimitiveType) type) : (type instanceof ASTReferenceType ? this.doPrintReferenceTypeWithoutTypeArguments((ASTReferenceType) type) : ""));
  }

  public static String printTypeWithoutTypeArgumentsAndDimension(ASTType type) {
    return getInstance().doPrintTypeWithoutTypeArgumentsAndDimension(type);
  }

  protected String doPrintTypeWithoutTypeArgumentsAndDimension(ASTType type) {

    if (type instanceof ASTArrayType)
      return this.doPrintTypeWithoutTypeArgumentsAndDimension(((ASTArrayType) type).getComponentType());

    if (type instanceof ASTPrimitiveType)
      return this.doPrintPrimitiveType((ASTPrimitiveType) type);

    if (type instanceof ASTReferenceType)
      return this.doPrintTypeWithoutTypeArguments(type);

    return resolveNewType(type);
  }

  protected String resolveNewType(ASTType type) {
    if (type instanceof ASTPrintType) {
      return ((ASTPrintType) type).printType();
    }
    Log.error("Type can not be handled!");
    return "";
  }

  public static String printTypeParameters(ASTTypeParameters params) {
    return getInstance().doPrintTypeParameters(params);
  }

  protected String doPrintTypeParameters(ASTTypeParameters params) {
    return params != null && params.getTypeVariableDeclarations() != null && !params.getTypeVariableDeclarations().isEmpty() ? "<" + this.doPrintTypeVariableDeclarationList(params.getTypeVariableDeclarations()) + ">" : "";
  }

  public static String printTypeVariableDeclarationList(List<ASTTypeVariableDeclaration> decl) {
    return getInstance().doPrintTypeVariableDeclarationList(decl);
  }

  protected String doPrintTypeVariableDeclarationList(List<ASTTypeVariableDeclaration> decl) {
    StringBuilder ret = new StringBuilder();
    if (decl != null) {
      String sep = "";

      for (Iterator var4 = decl.iterator(); var4.hasNext(); sep = ", ") {
        ASTTypeVariableDeclaration d = (ASTTypeVariableDeclaration) var4.next();
        ret.append(sep + this.doPrintTypeVariableDeclaration(d));
      }
    }

    return ret.toString();
  }

  public static String printTypeVariableDeclaration(ASTTypeVariableDeclaration decl) {
    return getInstance().doPrintTypeVariableDeclaration(decl);
  }

  protected String doPrintTypeVariableDeclaration(ASTTypeVariableDeclaration decl) {
    StringBuilder ret = new StringBuilder();
    if (decl != null) {
      ret.append(decl.getNamingResolution().get().getName());
      if (decl.getUpperBounds() != null && !decl.getUpperBounds().isEmpty()) {
        String sep = " extends ";

        for (Iterator var4 = decl.getUpperBounds().iterator(); var4.hasNext(); sep = " & ") {
          ASTType type = (ASTType) var4.next();
          ret.append(sep + this.doPrintType(type));
        }
      }
    }

    return ret.toString();
  }

  public static String printVoidType(ASTVoidType type) {
    return getInstance().doPrintVoidType(type);
  }

  protected String doPrintVoidType(ASTVoidType type) {
    return type != null ? "void" : "";
  }

  public static String printPrimitiveType(ASTPrimitiveType type) {
    return getInstance().doPrintPrimitiveType(type);
  }

  protected String doPrintPrimitiveType(ASTPrimitiveType type) {
    if (type == null)
      return "";
    if (type.getPrimitive() == 1)
      return "boolean";
    if (type.getPrimitive() == 2)
      return "byte";
    if (type.getPrimitive() == 3)
      return "short";
    if (type.getPrimitive() == 4)
      return "int";
    if (type.getPrimitive() == 5)
      return "long";
    if (type.getPrimitive() == 6)
      return "char";
    if (type.getPrimitive() == 7)
      return "float";
    if (type.getPrimitive() == 8)
      return "double";

    return "";
  }

  public static String printArrayType(ASTArrayType type) {
    return getInstance().doPrintArrayType(type);
  }

  protected String doPrintArrayType(ASTArrayType type) {
    if (type == null) {
      return "";
    }
    else {
      StringBuilder dimension = new StringBuilder();
      dimension.append(this.doPrintType(type.getComponentType()));

      for (int i = 0; i < type.getDimensions(); ++i) {
        dimension.append("[]");
      }

      return dimension.toString();
    }
  }

  public static String printReferenceTypeList(List<ASTReferenceType> type) {
    return getInstance().doPrintReferenceTypeList(type);
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

  public static String printSimpleReferenceType(ASTSimpleReferenceType type) {
    return getInstance().doPrintSimpleReferenceType(type);
  }

  protected String doPrintSimpleReferenceType(ASTSimpleReferenceType type) {
    return type != null ? (type.getTypeArguments().isPresent() ? Names.getQualifiedName(type.getNames()) + this.doPrintTypeArguments((ASTTypeArguments) type.getTypeArguments().get()) : Names.getQualifiedName(type.getNames())) : "";
  }

  public static String printComplexReferenceType(ASTComplexReferenceType type) {
    return getInstance().doPrintComplexReferenceType(type);
  }

  protected String doPrintComplexReferenceType(ASTComplexReferenceType type) {
    String ret = "";
    return type != null && type.getSimpleReferenceTypes() != null ? this.doPrintSimpleReferenceTypeList(type.getSimpleReferenceTypes()) : ret;
  }

  public static String printSimpleReferenceTypeList(List<ASTSimpleReferenceType> type) {
    return getInstance().doPrintSimpleReferenceTypeList(type);
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

  public static String printTypeArguments(ASTTypeArguments args) {
    return getInstance().doPrintTypeArguments(args);
  }

  protected String doPrintTypeArguments(ASTTypeArguments args) {
    return args != null && args.getTypeArguments() != null && !args.getTypeArguments().isEmpty() ? "<" + this.doPrintTypeArgumentList(args.getTypeArguments()) + ">" : "";
  }

  public static String printTypeArgumentList(List<ASTTypeArgument> argList) {
    return getInstance().doPrintTypeArgumentList(argList);
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

  public static String printWildcardType(ASTWildcardType type) {
    return getInstance().doPrintWildcardType(type);
  }

  protected String doPrintWildcardType(ASTWildcardType type) {
    StringBuilder ret = new StringBuilder();
    if (type != null) {
      ret.append("?");
      if (type.getUpperBound().isPresent()) {
        ret.append(" extends " + this.doPrintType((ASTType) type.getUpperBound().get()));
      }
      else if (type.getLowerBound().isPresent()) {
        ret.append(" super " + this.doPrintType((ASTType) type.getLowerBound().get()));
      }
    }

    return ret.toString();
  }

  protected String doPrintReferenceTypeWithoutTypeArguments(ASTReferenceType type) {
    return type instanceof ASTSimpleReferenceType ? this.doPrintSimpleReferenceTypeWithoutTypeArguments((ASTSimpleReferenceType) type) : (type instanceof ASTComplexReferenceType ? this.doPrintComplexReferenceTypeWithoutTypeArguments((ASTComplexReferenceType) type) : "");
  }

  protected String doPrintSimpleReferenceTypeWithoutTypeArguments(ASTSimpleReferenceType type) {
    return type != null ? Names.getQualifiedName(type.getNames()) : "";
  }

  protected String doPrintComplexReferenceTypeWithoutTypeArguments(ASTComplexReferenceType type) {
    return type != null && type.getSimpleReferenceTypes() != null ? this.doPrintSimpleReferenceTypeListWithoutTypeArguments(type.getSimpleReferenceTypes()) : "";
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
