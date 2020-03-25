/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ActualTypeArgumentASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesPrinter;
import de.monticore.lang.monticar.resolution._ast.ASTNamingResolution;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.MontiCarSymbolFactory;
import de.monticore.lang.monticar.ts.MontiCarTypeSymbol;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.types.types._ast.*;
import de.se_rwth.commons.logging.Log;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * TODO This class should be removed by putting its methods in JavaDSL (or even MC/Types) project.
 *
 */
public class EMAJavaHelper {
    private final static MontiCarSymbolFactory jSymbolFactory = new MontiCarSymbolFactory();

    /**
     * Adds the TypeParameters to the MontiCarTypeSymbol if the class or interface declares TypeVariables.
     * Example:
     * <p>
     * class Bla<T, S extends SomeClass<T> & SomeInterface>
     * </p>
     * T and S are added to Bla.
     *
     * @param typeSymbol
     * @param optionalTypeParameters
     * @return MontiCarTypeSymbol list to be added to the scope
     */
    // TODO see JavaSymbolTableCreator.addTypeParameters(...),
    // see EMAComponentSymbol addFormalTypeParameters etc.
    public static List<MCTypeSymbol> addTypeParametersToType(
            EMAComponentSymbol typeSymbol,
            ASTTypeParameters2 optionalTypeParameters, Scope currentScope) {

        ASTTypeParameters2 astTypeParameters = optionalTypeParameters;
        for (ASTTypeVariableDeclaration2 astTypeParameter : astTypeParameters.getTypeVariableDeclaration2List()) {
            if (astTypeParameter.getResolutionDeclarationOpt().isPresent()) {
                //Not handled here
                // new type parameter

                // TypeParameters/TypeVariables are seen as type declarations.
                // For each variable instantiate a MontiCarTypeSymbol.
                //TODO FIX if not present
                if (astTypeParameter.getResolutionDeclaration().getTypeName() != null) {
                    final String typeVariableName = astTypeParameter.getResolutionDeclaration().getTypeName();

                    addFormalTypeParameter(typeVariableName, astTypeParameter, currentScope, typeSymbol);
                } else if (astTypeParameter.getResolutionDeclaration() instanceof ASTNamingResolution) {

                    final String typeVariableName = astTypeParameter.getResolutionDeclaration().getName();
                    addFormalTypeParameter(typeVariableName, astTypeParameter, currentScope, typeSymbol);
                } else {
                    Log.debug(astTypeParameter.getResolutionDeclaration().toString(), "Resolution Declaration");
                    Log.debug("0xADTYPA Case not handled", "Implementation Missing");
                }
            } else {
                final String typeVariableName = astTypeParameter.getNamingResolution().getName();
                addFormalTypeParameter(typeVariableName, astTypeParameter, currentScope, typeSymbol);
            }
        }
        return typeSymbol.getFormalTypeParameters();
    }

    private static void addFormalTypeParameter(String typeVariableName, ASTTypeVariableDeclaration2
            astTypeParameter, Scope currentScope, EMAComponentSymbol typeSymbol) {
        MontiCarTypeSymbol javaTypeVariableSymbol = jSymbolFactory.createTypeVariable(typeVariableName);
        // TODO implement
        // // init type parameter
        // if (astTypeParameter.getTypeBound().isPresent()) {
        // // Treat type bounds are implemented interfaces, even though the first
        // // bound might be a class. See also JLS7.
        // addInterfacesToType(javaTypeVariableSymbol, astTypeParameter.getTypeBound().get()
        // .getTypes());
        // }
        // Treat type bounds are implemented interfaces, even though the
        // first bound might be a class. See also JLS7.
        List<ASTType> types = new ArrayList<ASTType>(astTypeParameter.getUpperBoundsList());

        addInterfacesToTypeEMA(javaTypeVariableSymbol, types, currentScope);

        // add type parameter
        typeSymbol.addFormalTypeParameter(javaTypeVariableSymbol);
    }

    /**
     * Adds the given ASTTypes as interfaces to the MontiCarTypeSymbol. The MontiCarTypeSymbol can be a type
     * variable. Interfaces may follow after the first extended Type. We treat the first Type also as
     * interface even though it may be a class.
     * <p>
     * class Bla implements SomeInterface, AnotherInterface, ... <br>
     * class Bla&ltT extends SomeClassOrInterface & SomeInterface & ...&gt
     * </p>
     * See also JLS7.
     *
     * @param astInterfaceTypeList
     */
    // TODO this is implemented in JavaDSL, but reimplemented because of ArcTypeSymbol. This should
    // somehow be extracted and implemented only once
    protected static void addInterfacesToTypeEMA(MontiCarTypeSymbol arcTypeSymbol,
                                                 List<ASTType> astInterfaceTypeList, Scope currentScope) {
        for (ASTType astInterfaceType : astInterfaceTypeList) {
            MontiCarTypeSymbolReference javaInterfaceTypeSymbolReference = new MontiCarTypeSymbolReference(
                    TypesPrinter.printTypeWithoutTypeArgumentsAndDimension(astInterfaceType), currentScope,
                    0);
            List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();

            // Add the ASTTypeArguments to astInterfaceType
            // Before we can do that we have to cast.
            if (astInterfaceType instanceof ASTSimpleReferenceType) {
                // TODO
                // addTypeParametersToType(javaInterfaceTypeSymbolReference, astInterfaceType);
            } else if (astInterfaceType instanceof ASTComplexReferenceType) {
                ASTComplexReferenceType astComplexReferenceType = (ASTComplexReferenceType) astInterfaceType;
                for (ASTSimpleReferenceType astSimpleReferenceType : astComplexReferenceType
                        .getSimpleReferenceTypeList()) {
                    // TODO javaInterfaceTypeSymbolReference.getEnclosingScope().resolve("Boolean", MCTypeSymbol.KIND).get()
                    //    javaInterfaceTypeSymbolReference.getEnclosingScope().resolve("Boolean", MCTypeSymbol.KIND))
                    if (astSimpleReferenceType.getTypeArgumentsOpt().isPresent()) {
                        for (ASTTypeArgument argument : astSimpleReferenceType.getTypeArgumentsOpt().get().getTypeArgumentList()) {

                            if (!handleSimpleReferenceType(argument, 0, actualTypeArguments,
                                    javaInterfaceTypeSymbolReference.getEnclosingScope(), false, false, null) &&
                                    (argument instanceof ASTComplexArrayType)) {
                                ASTComplexArrayType arrayArg = (ASTComplexArrayType) argument;
                                ASTType cmpType = arrayArg.getComponentType();
                                handleSimpleReferenceType(cmpType, arrayArg.getDimensions(),
                                        actualTypeArguments, javaInterfaceTypeSymbolReference.getEnclosingScope(), false, false, null);
                            }
                        }

                    }

                    arcTypeSymbol.addInterface(javaInterfaceTypeSymbolReference);
                }
            }
            javaInterfaceTypeSymbolReference.setActualTypeArguments(actualTypeArguments);
        }
    }

    /**
     * if you have questions ask JP ;)
     */
    protected static boolean handleSimpleReferenceType(ASTTypeArgument argument,
                                                       final int dim, final List<ActualTypeArgument> actualTypeArguments, final Scope symbolTable,
                                                       final boolean isLowerBound, final boolean isUpperBound, @Nullable ActualTypeArgument typeArgument) {
        if (argument instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType simpleArg = (ASTSimpleReferenceType) argument;
            String name = simpleArg.getNameList().stream().collect(Collectors.joining("."));
            Optional<MCTypeSymbol> symbol = symbolTable.resolve(name, MCTypeSymbol.KIND);
            if (symbol.isPresent() && symbol.get().getEnclosingScope() != null) {
                if (typeArgument == null) {
                    typeArgument = new ActualTypeArgumentASTElement(isLowerBound, isUpperBound, new MontiCarTypeSymbolReference(
                            symbol.get().getName(),
                            symbol.get().getEnclosingScope(), dim)).setAstTypeArguments(argument);

                    actualTypeArguments.add(typeArgument);
                }
                if (simpleArg.getTypeArgumentsOpt().isPresent()) {
                    List<ActualTypeArgument> actualTypeArguments2 = new ArrayList<>();
                    for (ASTTypeArgument astTypeArgument : simpleArg.getTypeArgumentsOpt().get().getTypeArgumentList()) {
                        handleSimpleReferenceType(astTypeArgument, 0, actualTypeArguments2, symbolTable, false, false, typeArgument);
                    }
                    typeArgument.getType().setActualTypeArguments(actualTypeArguments2);
                }
            }
            return true;
        } else if (argument instanceof ASTComplexReferenceType) {
            ASTComplexReferenceType complexArg = (ASTComplexReferenceType) argument;
            complexArg.getSimpleReferenceTypeList().stream().forEachOrdered(t ->
                    handleSimpleReferenceType(t, dim, actualTypeArguments, symbolTable, isLowerBound, isUpperBound, null)
            );
            return true;
        } else if (argument instanceof ASTWildcardType) {
            ASTWildcardType wildArg = (ASTWildcardType) argument;
            if (wildArg.getLowerBoundOpt().isPresent()) {
                return handleSimpleReferenceType(wildArg.getLowerBound(), dim, actualTypeArguments,
                        symbolTable, true, false, null);
            } else if (wildArg.getUpperBoundOpt().isPresent()) {
                return handleSimpleReferenceType(wildArg.getUpperBound(), dim, actualTypeArguments,
                        symbolTable, false, true, null);
            }
        } else if (argument instanceof ASTComplexArrayType) {
            ASTComplexArrayType arrayArg = (ASTComplexArrayType) argument;
            ASTType cmpType = arrayArg.getComponentType();
            return handleSimpleReferenceType(cmpType, arrayArg.getDimensions(),
                    actualTypeArguments, symbolTable, false, false, null);
        }
        return false;
    }

    // TODO this should be part of JavaDSL
    public static void addJavaDefaultTypes(GlobalScope globalScope) {
        // we add default types by putting mock implementations of java.lang and java.util in
        // src/main/resources/defaultTypes and adding it to the model path when creating the global scope!
        // TODO This, however, should be done in JavaDSL and the EmbeddedMontiArcLanguage somehow...
    }

    /**
     * Adds the default imports of the java language to make default types resolvable without
     * qualification (e.g., "String" instead of "java.lang.String").
     *
     * @param imports
     */
    public static void addJavaDefaultImports(List<ImportStatement> imports) {
        imports.add(new ImportStatement("java.lang", true));
        imports.add(new ImportStatement("java.util", true));
    }
}
