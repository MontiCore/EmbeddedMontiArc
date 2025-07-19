/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.contract;

import static de.monticore.lang.monticar.contract.Precondition.requires;
import static de.monticore.lang.monticar.contract.StringPrecondition.Condition.notBlank;
import static de.monticore.lang.monticar.contract.StringPrecondition.Condition.notEmpty;

import de.monticore.lang.monticar.contract.Precondition.PreconditionViolationException;
import java.util.function.Supplier;

@SuppressWarnings("WeakerAccess")
public class StringPrecondition {

  /**
   * Assert that the supplied {@link String} is not empty.
   * <p>
   * <p>A {@code String} is <em>empty</em> if it is {@code null} or the empty string.
   *
   * @param charSequence the string to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied string as a convenience
   * @throws PreconditionViolationException if the supplied string is blank
   * @see Condition#notEmpty(CharSequence)
   * @see Precondition#requires(boolean, Supplier)
   */
  public static <T extends CharSequence> T requiresNotEmpty(final T charSequence,
      final Supplier<String> messageSupplier) {
    requires(notEmpty(charSequence), messageSupplier);
    return charSequence;
  }

  /**
   * @see #requiresNotEmpty(CharSequence, Supplier)
   */
  public static <T extends CharSequence> T requiresNotEmpty(final T charSequence) {
    return requiresNotEmpty(charSequence, () -> NOT_BLANK_STRING_VIOLATION);
  }

  /**
   * Assert that the supplied {@link String} is not blank.
   * <p>
   * <p>A {@code String} is <em>blank</em> if it is {@code null}, empty, or consists
   * only of whitespace characters.
   *
   * @param charSequence the string to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied string as a convenience
   * @throws PreconditionViolationException if the supplied string is blank
   * @see Condition#notBlank(CharSequence)
   * @see Precondition#requires(boolean, Supplier)
   */
  public static <T extends CharSequence> T requiresNotBlank(final T charSequence,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notBlank(charSequence), messageSupplier);
    return charSequence;
  }

  /**
   * @see #requiresNotBlank(CharSequence, Supplier)
   */
  public static <T extends CharSequence> T requiresNotBlank(final T charSequence)
      throws PreconditionViolationException {
    return requiresNotBlank(charSequence, () -> NOT_BLANK_STRING_VIOLATION);
  }

  private static final String NOT_EMPTY_STRING_VIOLATION = "Requires string to be not null or empty.";
  private static final String NOT_BLANK_STRING_VIOLATION = "Requires string to be not null, empty, nor consisting of only whitespace characters.";

  public static final class Condition {

    /**
     * Checks whether the given {@code charSequence} is not empty (length > 0).
     * <p>
     * Note that {@code null}, logically, has no length, and thus, does not fulfill this
     * condition.
     *
     * @param charSequence the parameter to check
     * @return true iff {@code charSequence} is not {@code null} nor empty (length > 0)
     */
    public static boolean notEmpty(final CharSequence charSequence) {
      return null != charSequence && charSequence.length() > 0;
    }

    /**
     * Checks whether the given {@code charSequence} is {@link #notEmpty(CharSequence)} nor
     * comprises only whitespaces.
     *
     * @param charSequence the parameter to check
     * @return true iff {@code charSequence} is not empty nor empty comprises only whitespaces
     * @see #notEmpty(CharSequence)
     */
    public static boolean notBlank(final CharSequence charSequence) {
      return notEmpty(charSequence) && !charSequence.toString().trim().isEmpty();
    }
  }
}
