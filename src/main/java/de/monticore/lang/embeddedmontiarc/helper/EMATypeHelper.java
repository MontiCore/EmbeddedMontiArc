/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ActualTypeArgumentNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesHelper;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ranges._ast.ASTRanges;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbolReference;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCASTTypeSymbolReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.types.types._ast.*;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class EMATypeHelper {

    /**
     * handles typeref creation of an SIUnitRangeType from ASTRange
     */
    public static MCTypeReference<? extends MCTypeSymbol> initTypeRefASTRange(StringBuilder typeName,
                                                                              ASTRange astType) {
        typeName.append("SIUnitRangesType");
        Log.debug(astType.toString(), "Type:");
        Log.debug(typeName.toString(), "TypeName:");

        SIUnitRangesSymbolReference ref = SIUnitRangesSymbolReference
                .constructSIUnitRangesSymbolReference(astType);
        return ref;
    }

    /**
     * handles typeref creation of an SIUnitRangesType from ASTRanges
     */
    public static MCTypeReference<? extends MCTypeSymbol> initTypeRefASTRanges(StringBuilder typeName,
                                                                               ASTRanges astType) {
        typeName.append("SIUnitRangesType");
        Log.debug(astType.toString(), "Type:");
        Log.debug(typeName.toString(), "TypeName:");

        SIUnitRangesSymbolReference ref = SIUnitRangesSymbolReference
                .constructSIUnitRangesSymbolReference(astType.getRangeList());
        return ref;
    }

    /**
     * handles typeref creation of a GeneralType
     */
    public static MCTypeReference<? extends MCTypeSymbol> initTypeRefGeneralType(StringBuilder typeName,
                                                                                 ASTType astType,
                                                                                 EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        MCTypeReference<? extends MCTypeSymbol> typeRef = null;
        typeName.append(ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(astType));
        // Log.debug(astType.toString(),"TYPE:");
        // Log.debug(typeName,"TYPEName:");
        typeRef = new CommonMCTypeReference<MCTypeSymbol>(typeName.toString(), MCTypeSymbol.KIND,
                symbolTableCreator.currentScope().get());
        typeRef.setDimension(TypesHelper.getArrayDimensionIfArrayOrZero(astType));
        typeRef = addTypeArgumentsToTypeSymbol(typeRef, astType, symbolTableCreator);
        return typeRef;
    }

    /**
     * handles typeref creation of a GeneralType
     */
    public static MCTypeReference<? extends MCTypeSymbol> initTypeRefGeneralType(String typeName,
                                                                                 EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        MCTypeReference<? extends MCTypeSymbol> typeRef = null;
        typeRef = new CommonMCTypeReference<MCTypeSymbol>(typeName, MCTypeSymbol.KIND,
                symbolTableCreator.currentScope().get());
        typeRef.setDimension(0);
        // addTypeArgumentsToTypeSymbol(typeRef, astType);
        return typeRef;
    }

    /**
     * returns an initialized type reference for a port
     *
     * @param node     the node which is
     * @param typeName
     * @param astType
     * @return
     */
    public static MCTypeReference<? extends MCTypeSymbol> initTypeRef(ASTPort node,
                                                                      StringBuilder typeName, ASTType astType
            , EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        if (node.getType() instanceof ASTRange) {
            return initTypeRefASTRange(typeName, (ASTRange) astType);
        } else if (node.getType() instanceof ASTRanges) {
            return initTypeRefASTRanges(typeName, (ASTRanges) astType);
        }
        Log.debug(node.getNameOpt().get() + " " + astType.toString(), "info");
        return initTypeRefGeneralType(typeName, astType, symbolTableCreator);
    }


    // TODO remove after GV's refactoring of such methodology to mc4/types.
    @Deprecated
    public static MCTypeReference<? extends MCTypeSymbol> addTypeArgumentsToTypeSymbol(MCTypeReference<? extends MCTypeSymbol> typeReference,
                                                                                       ASTType astType,
                                                                                       EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        if (astType instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType astSimpleReferenceType = (ASTSimpleReferenceType) astType;
            if (!astSimpleReferenceType.getTypeArgumentsOpt().isPresent()) {
                return typeReference;
            }
            List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();
            for (ASTTypeArgument astTypeArgument : astSimpleReferenceType.getTypeArgumentsOpt().get()
                    .getTypeArgumentList()) {
                if (astTypeArgument instanceof ASTWildcardType) {
                    ASTWildcardType astWildcardType = (ASTWildcardType) astTypeArgument;

                    // Three cases can occur here: lower bound, upper bound, no bound
                    if (astWildcardType.getLowerBoundOpt().isPresent() || astWildcardType.getUpperBoundOpt().isPresent()) {
                        // We have a bound.
                        // Examples: Set<? extends Number>, Set<? super Integer>

                        // new bound
                        boolean lowerBound = astWildcardType.getLowerBoundOpt().isPresent();
                        ASTType typeBound = lowerBound
                                ? astWildcardType.getLowerBound()
                                : astWildcardType
                                .getUpperBound();

                        int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(typeBound);
                        MCTypeReference<? extends MCTypeSymbol> typeBoundSymbolReference = new MontiCarTypeSymbolReference(
                                ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(typeBound),
                                symbolTableCreator.currentScope().get(), dimension);
                        // TODO string representation?
                        // typeBoundSymbolReference.setStringRepresentation(ArcTypePrinter
                        // .printWildcardType(astWildcardType));
                        ActualTypeArgument actualTypeArgument = new ActualTypeArgument(lowerBound, !lowerBound,
                                typeBoundSymbolReference);

                        // init bound
                        addTypeArgumentsToTypeSymbol(typeBoundSymbolReference, typeBound, symbolTableCreator);

                        actualTypeArguments.add(actualTypeArgument);
                    } else {
                        // No bound. Example: Set<?>
                        actualTypeArguments.add(new ActualTypeArgument(false, false, null));
                    }
                } else if (astTypeArgument instanceof ASTType) {
                    // Examples: Set<Integer>, Set<Set<?>>, Set<java.lang.String>
                    ASTType astTypeNoBound = (ASTType) astTypeArgument;
                    int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(astTypeNoBound);
                    MCTypeReference<? extends MCTypeSymbol> typeArgumentSymbolReference = new MontiCarTypeSymbolReference(
                            ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(astTypeNoBound),
                            symbolTableCreator.currentScope().get(), dimension);

                    // TODO string representation?
                    // typeArgumentSymbolReference.setStringRepresentation(TypesPrinter
                    // .printType(astTypeNoBound));

                    addTypeArgumentsToTypeSymbol(typeArgumentSymbolReference, astTypeNoBound, symbolTableCreator);

                    actualTypeArguments
                            .add(new ActualTypeArgumentNode(typeArgumentSymbolReference, astTypeNoBound));
                } else {
                    Log.error("0xU0401 Unknown type arguments " + astTypeArgument + " of type "
                            + typeReference);
                }
                typeReference.setActualTypeArguments(actualTypeArguments);
            }
        } else if (astType instanceof ASTComplexReferenceType) {
            ASTComplexReferenceType astComplexReferenceType = (ASTComplexReferenceType) astType;
            for (ASTSimpleReferenceType astSimpleReferenceType : astComplexReferenceType
                    .getSimpleReferenceTypeList()) {
                // TODO
                /* ASTComplexReferenceType represents types like class or interface types which always have
                 * ASTSimpleReferenceType as qualification. For example: a.b.c<Arg>.d.e<Arg> */
            }
        } else if (astType instanceof ASTComplexArrayType) {
            ASTComplexArrayType astComplexArrayType = (ASTComplexArrayType) astType;
            // references to types with dimension>0, e.g., String[]
            addTypeArgumentsToTypeSymbol(typeReference, astComplexArrayType.getComponentType(), symbolTableCreator);
            int dimension = astComplexArrayType.getDimensions();
            typeReference.setDimension(dimension);
        } else if (astType instanceof ASTElementType) {
            return typeReference;
        } else {
            String name = typeReference.getName();
            assert typeReference.getEnclosingScope() instanceof MutableScope;
            MutableScope enclosingScope = (MutableScope) typeReference.getEnclosingScope();
            typeReference = MCASTTypeSymbolReference.constructReference(name, enclosingScope, astType);
        }
        return typeReference;
    }

    public static void setActualTypeArguments(EMAComponentSymbolReference typeReference,
                                              List<ASTTypeArgument> astTypeArguments,
                                              EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        List<ActualTypeArgument> actualTypeArguments = new ArrayList<>();
        for (ASTTypeArgument astTypeArgument : astTypeArguments) {
            if (astTypeArgument instanceof ASTWildcardType) {
                ASTWildcardType astWildcardType = (ASTWildcardType) astTypeArgument;

                // Three cases can occur here: lower bound, upper bound, no bound
                if (astWildcardType.getLowerBoundOpt().isPresent() || astWildcardType.getUpperBoundOpt().isPresent()) {
                    // We have a bound.
                    // Examples: Set<? extends Number>, Set<? super Integer>

                    // new bound
                    boolean lowerBound = astWildcardType.getLowerBoundOpt().isPresent();
                    ASTType typeBound = lowerBound
                            ? astWildcardType.getLowerBound()
                            : astWildcardType
                            .getUpperBound();
                    int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(typeBound);
                    MCTypeReference<? extends MCTypeSymbol> typeBoundSymbolReference = new MontiCarTypeSymbolReference(
                            ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(typeBound),
                            symbolTableCreator.currentScope().get(), dimension);
                    // TODO string representation?
                    // typeBoundSymbolReference.setStringRepresentation(ArcTypePrinter
                    // .printWildcardType(astWildcardType));
                    ActualTypeArgument actualTypeArgument = new ActualTypeArgument(lowerBound, !lowerBound,
                            typeBoundSymbolReference);

                    // init bound
                    addTypeArgumentsToTypeSymbol(typeBoundSymbolReference, typeBound, symbolTableCreator);

                    actualTypeArguments.add(actualTypeArgument);
                } else {
                    // No bound. Example: Set<?>
                    actualTypeArguments.add(new ActualTypeArgument(false, false, null));
                }
            } else if (astTypeArgument instanceof ASTType) {
                // Examples: Set<Integer>, Set<Set<?>>, Set<java.lang.String>
                ASTType astTypeNoBound = (ASTType) astTypeArgument;
                int dimension = TypesHelper.getArrayDimensionIfArrayOrZero(astTypeNoBound);
                MCTypeReference<? extends MCTypeSymbol> typeArgumentSymbolReference = new MontiCarTypeSymbolReference(
                        ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(astTypeNoBound),
                        symbolTableCreator.currentScope().get(), dimension);

                // TODO string representation?
                // typeArgumentSymbolReference.setStringRepresentation(TypesPrinter
                // .printType(astTypeNoBound));

                typeArgumentSymbolReference = addTypeArgumentsToTypeSymbol(typeArgumentSymbolReference, astTypeNoBound, symbolTableCreator);

                actualTypeArguments.add(new ActualTypeArgument(typeArgumentSymbolReference));
            } else {
                Log.error("0xU0401 Unknown type arguments " + astTypeArgument + " of type "
                        + typeReference);
            }
        }
        typeReference.setActualTypeArguments(actualTypeArguments);
    }

    // TODO references to component symbols should not differ from MontiCarTypeSymbolReference?
    @Deprecated
    public static void addTypeArgumentsToTypeSymbol(EMAComponentSymbolReference typeReference,
                                                    ASTType astType, EmbeddedMontiArcSymbolTableCreator symbolTableCreator) {
        if (astType instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType astSimpleReferenceType = (ASTSimpleReferenceType) astType;
            if (!astSimpleReferenceType.getTypeArgumentsOpt().isPresent()) {
                // Log.error("Not TypeArgs present");
                return;
            }
            setActualTypeArguments(typeReference,
                    astSimpleReferenceType.getTypeArgumentsOpt().get().getTypeArgumentList(), symbolTableCreator);
        } else if (astType instanceof ASTComplexReferenceType) {
            ASTComplexReferenceType astComplexReferenceType = (ASTComplexReferenceType) astType;
            for (ASTSimpleReferenceType astSimpleReferenceType : astComplexReferenceType
                    .getSimpleReferenceTypeList()) {
                // TODO
                /* ASTComplexReferenceType represents types like class or interface types which always have
                 * ASTSimpleReferenceType as qualification. For example: a.b.c<Arg>.d.e<Arg> */

            }
        }

    }
}
