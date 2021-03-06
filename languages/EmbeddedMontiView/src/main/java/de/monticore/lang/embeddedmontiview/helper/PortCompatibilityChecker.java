/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.helper;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewPortSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;

import java.util.List;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Checks type compatibility of connected ports.
 *
 */
public class PortCompatibilityChecker {
  /**
   * Checks whether the sourcePort's type can be connected to the targetPort's type. For example,
   * consider a generic subcomponent {@code s<Int>} of type {@code S<T>} with a port {@code p} of
   * type {@code T}. If {@code p} is the sourcePort, the sourceFormalTypeParameters list is
   * {@code [T]}, the sourceTypeArguments is {@code [Int]}.<br>
   * <br>
   * This type-check would allow a typed based auto-connection of {@code aOut} with {@code p} in the
   * following example:
   * <pre>
   * component A {
   *   port out Int aOut;
   *   component S&lt;T&gt; s&lt;Int&gt; {
   *     port out T p;
   *   }
   * }
   * </pre>
   *
   * @param sourcePort                     the port that outputs data
   * @param sourceFormalTypeParameters     the defined formal type parameters of the component that the
   *                                       sourcePort is defined in. They define additional valid types that might be bound by the
   *                                       sourceTypeArguments. This list might be empty.
   * @param sourceTypeArguments            Defines the current bindings for the formal type-parameters. This
   *                                       list might be empty.
   * @param targetPort                     the port that receives data
   * @param targetTypeFormalTypeParameters analog to source, but for the target port.
   * @param targetTypeArguments            analog to source, but for the target port.
   * @return
   */
  public static boolean doPortTypesMatch(ViewPortSymbol sourcePort, List<MCTypeSymbol> sourceFormalTypeParameters, List<MCTypeReference<? extends MCTypeSymbol>> sourceTypeArguments, ViewPortSymbol targetPort, List<MCTypeSymbol> targetTypeFormalTypeParameters, List<MCTypeReference<? extends MCTypeSymbol>> targetTypeArguments) {
    checkNotNull(sourcePort);
    checkNotNull(targetPort);
    //this might not work, but it should not be relevant for views since autoconnect is not used afaik
    if (sourcePort.isTypeAnonymous() || targetPort.isTypeAnonymous())
      return true;
    return TypeCompatibilityChecker.doTypesMatch(sourcePort.getTypeReference().get(), sourceFormalTypeParameters, sourceTypeArguments, targetPort.getTypeReference().get(), targetTypeFormalTypeParameters, targetTypeArguments);
  }

}
