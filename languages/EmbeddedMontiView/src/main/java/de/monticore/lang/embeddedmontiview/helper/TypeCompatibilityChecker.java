/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.helper;

import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.ts.references.MontiCarTypeSymbolReference;
import de.monticore.symboltable.types.references.ActualTypeArgument;

import java.util.List;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Checks type compatibility of {@link ArcTypeReference}s.
 *
 */
public class TypeCompatibilityChecker {
  private static int getPositionInFormalTypeParameters(List<MCTypeSymbol> formalTypeParameters, MCTypeReference<? extends MCTypeSymbol> searchedFormalTypeParameter) {
    int positionInFormal = 0;
    for (MCTypeSymbol formalTypeParameter : formalTypeParameters) {
      if (formalTypeParameter.getName().equals(searchedFormalTypeParameter.getName())) {
        break;
      }
      positionInFormal++;
    }
    return positionInFormal;
  }

  /**
   * Checks compatibility of {@link ArcTypeReference}s. The sourceTypeFomalTypeParameters list all
   * type parameters, while the sourceTypeArguments define the current binding of them. E.g., a
   * generic source Type {@code A<X, Y>} could be bound to
   * <code>{@code A<List<Optional<Integer>>, String>}</code>. For a targetType to match, it must
   * recursively match all generic bindings. In the example, the first recursion would check that
   * formal type-parameters of {@code List} are bound to the same type argument (here
   * {@code Optional}) for both, the source and the target type. The second recursion would check
   * {@code Optional}'s type arguments to be {@code Integer}. Then, the the other type-arguments of
   * {@code A} (here {@code Y}) are checked.
   *
   * @param sourceType
   * @param sourceTypeFormalTypeParameters
   * @param sourceTypeArguments
   * @param targetType
   * @param targetTypeFormalTypeParameters
   * @param targetTypeArguments
   * @return
   */
  public static boolean doTypesMatch(MCTypeReference<? extends MCTypeSymbol> sourceType, List<MCTypeSymbol> sourceTypeFormalTypeParameters, List<MCTypeReference<? extends MCTypeSymbol>> sourceTypeArguments, MCTypeReference<? extends MCTypeSymbol> targetType, List<MCTypeSymbol> targetTypeFormalTypeParameters, List<MCTypeReference<? extends MCTypeSymbol>> targetTypeArguments) {

    // TODO reuse Java type checker?

    checkNotNull(sourceType);
    checkNotNull(targetType);
    boolean result = false;
    if (sourceType.getReferencedSymbol().isFormalTypeParameter()) {
      // bind the generic to the actual type
      int positionInFormal = getPositionInFormalTypeParameters(sourceTypeFormalTypeParameters, sourceType);
      sourceType = sourceTypeArguments.get(positionInFormal);
    }
    if (targetType.getReferencedSymbol().isFormalTypeParameter()) {
      // bind the generic to the actual type
      int positionInFormal = getPositionInFormalTypeParameters(targetTypeFormalTypeParameters, targetType);
      targetType = targetTypeArguments.get(positionInFormal);
    }

    if (sourceType.getReferencedSymbol().getFullName().equals(targetType.getReferencedSymbol().getFullName()) && sourceType.getDimension() == targetType.getDimension() && sourceType.getActualTypeArguments().size() == targetType.getActualTypeArguments().size()) {
      result = true;
      // type without generics does match, now we must check that the type arguments match
      List<ActualTypeArgument> sourceParams = sourceType.getActualTypeArguments();
      List<ActualTypeArgument> targetParams = targetType.getActualTypeArguments();
      for (int i = 0; i < sourceParams.size(); i++) {
        MCTypeReference<? extends MCTypeSymbol> sourceTypesCurrentTypeArgument = (MCTypeReference<?>) sourceParams.get(i).getType();
        MCTypeReference<? extends MCTypeSymbol> targetTypesCurrentTypeArgument = (MCTypeReference<?>) targetParams.get(i).getType();
        if (!doTypesMatch(sourceTypesCurrentTypeArgument, sourceTypesCurrentTypeArgument.getReferencedSymbol().getFormalTypeParameters().stream().map(p -> (MCTypeSymbol) p).collect(Collectors.toList()), sourceTypesCurrentTypeArgument.getActualTypeArguments().stream().map(a -> (MCTypeReference<?>) a.getType()).collect(Collectors.toList()), targetTypesCurrentTypeArgument, targetTypesCurrentTypeArgument.getReferencedSymbol().getFormalTypeParameters().stream().map(p -> (MCTypeSymbol) p).collect(Collectors.toList()), targetTypesCurrentTypeArgument.getActualTypeArguments().stream().map(a -> (MCTypeReference<?>) a.getType()).collect(Collectors.toList()))) {
          result = false;
          break;
        }
      }
    }
    else if (!sourceType.getReferencedSymbol().getFullName().equals(targetType.getReferencedSymbol().getFullName())) {
      // check, if superclass from sourceType is compatible with targetType
      if (sourceType.getReferencedSymbol().getSuperClass().isPresent()) {
        MCTypeReference<? extends MCTypeSymbol> parent = sourceType.getReferencedSymbol().getSuperClass().get();
        result = doTypesMatch(parent, parent.getReferencedSymbol().getFormalTypeParameters().stream().map(p -> (MCTypeSymbol) p).collect(Collectors.toList()), parent.getActualTypeArguments().stream().map(a -> (MontiCarTypeSymbolReference) a.getType()).collect(Collectors.toList()), targetType, targetTypeFormalTypeParameters, targetTypeArguments);
      }
      if (!result && !sourceType.getReferencedSymbol().getInterfaces().isEmpty()) {
        for (MCTypeReference<? extends MCTypeSymbol> interf : sourceType.getReferencedSymbol().getInterfaces()) {
          result = doTypesMatch(interf, interf.getReferencedSymbol().getFormalTypeParameters().stream().map(p -> (MCTypeSymbol) p).collect(Collectors.toList()), interf.getActualTypeArguments().stream().map(a -> (MontiCarTypeSymbolReference) a.getType()).collect(Collectors.toList()), targetType, targetTypeFormalTypeParameters, targetTypeArguments);
          if (result) {
            break;
          }
        }
      }
    }
    return result;
  }

}