/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.contract;

import static de.monticore.lang.monticar.contract.Precondition.Condition.containsNoNulls;
import static de.monticore.lang.monticar.contract.Precondition.Condition.notEmpty;
import static de.monticore.lang.monticar.contract.Precondition.Condition.notNegative;
import static de.monticore.lang.monticar.contract.Precondition.Condition.notNull;

import java.util.Arrays;
import java.util.Collection;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Supplemental class for coding by contract.
 * <p>
 * A super simple class with a few static methods that allow for an easy-to-read, one-line
 * validation whether certain expectations (the contract) are met.
 * <p>
 */
@SuppressWarnings("WeakerAccess")
public final class Precondition {

  /**
   * Assert that the supplied {@code predicate} is {@code true}.
   *
   * @param predicate the predicate to check
   * @param messageSupplier precondition violation message supplier
   * @throws PreconditionViolationException if the predicate is {@code false}
   */
  public static void requires(final boolean predicate, final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(predicate, true, true, messageSupplier);
  }

  /**
   * @see #requires(boolean, Supplier)
   */
  public static void requires(final boolean predicate)
      throws PreconditionViolationException {
    requires(predicate, () -> null);
  }

  /**
   * Assert that the supplied {@code predicates} are {@code true}.
   *
   * @param predicate1 the first predicate to check
   * @param predicate2 the second predicate to check
   * @param messageSupplier precondition violation message supplier
   * @throws PreconditionViolationException if the predicate is {@code false}
   */
  public static void requires(final boolean predicate1,
      final boolean predicate2,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(predicate1, predicate2, true, messageSupplier);
  }

  /**
   * @see #requires(boolean, boolean, Supplier)
   */
  public static void requires(final boolean predicate1,
      final boolean predicate2)
      throws PreconditionViolationException {
    requires(predicate1, predicate2, () -> null);
  }

  /**
   * Assert that the supplied {@code predicates} are {@code true}.
   *
   * @param predicate1 the first predicate to check
   * @param predicate2 the second predicate to check
   * @param predicate3 the third predicate to check
   * @param messageSupplier precondition violation message supplier
   * @throws PreconditionViolationException if the predicate is {@code false}
   */
  public static void requires(final boolean predicate1,
      final boolean predicate2,
      final boolean predicate3,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    if (!predicate1 || !predicate2 || !predicate3) {
      throw new PreconditionViolationException(messageSupplier.get());
    }
  }

  /**
   * @see #requires(boolean, boolean, boolean, Supplier)
   */
  public static void requires(final boolean predicate1,
      final boolean predicate2,
      final boolean predicate3)
      throws PreconditionViolationException {
    requires(predicate1, predicate2, predicate3, () -> null);
  }

  /**
   * Assert that the supplied {@link Object} is not {@code null}.
   *
   * @param object the object to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied object as a convenience
   * @throws PreconditionViolationException if the supplied object is {@code null}
   * @see Condition#notNull(Object)
   * @see #requires(boolean, Supplier)
   */
  public static <T> T requiresNotNull(final T object, final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notNull(object), messageSupplier);
    return object;
  }

  /**
   * @see #requiresNotNull(Object, Supplier)
   */
  public static <T> T requiresNotNull(final T object)
      throws PreconditionViolationException {
    return requiresNotNull(object, () -> NOT_NULL_VIOLATION);
  }

  /**
   * Assert that the supplied array and none of the objects it contains are
   * {@code null}.
   *
   * @param objects the array to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied array as a convenience
   * @throws PreconditionViolationException if the supplied array or any of the objects its contains
   * is {@code null}
   * @see Condition#notNull(Object)
   * @see Condition#containsNoNulls(Object[])
   * @see #requires(boolean, boolean, Supplier)
   */
  public static Object[] requiresNotNullNoNulls(final Object[] objects,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notNull(objects), containsNoNulls(objects), messageSupplier);
    return objects;
  }

  /**
   * @see #requiresNotNullNoNulls(Object[], Supplier)
   */
  public static Object[] requiresNotNullNoNulls(final Object[] objects)
      throws PreconditionViolationException {
    return requiresNotNullNoNulls(objects, () -> NOT_NULL_NO_NULLS_WITH_ARRAY_VIOLATION);
  }

  /**
   * Assert that the supplied {@link Collection} and none of its entries are {@code null}.
   *
   * @param collection the collection to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied collection as a convenience
   * @throws PreconditionViolationException if the supplied collection or any of its entries is
   * {@code null}
   * @see Condition#notNull(Object)
   * @see Condition#containsNoNulls(Collection)
   * @see #requires(boolean, boolean, Supplier)
   */
  public static <T extends Collection<?>> T requiresNotNullNoNulls(final T collection,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notNull(collection), containsNoNulls(collection), messageSupplier);
    return collection;
  }

  /**
   * @see #requiresNotNullNoNulls(Collection, Supplier)
   */
  public static <T extends Collection<?>> T requiresNotNullNoNulls(final T collection)
      throws PreconditionViolationException {
    return requiresNotNullNoNulls(collection,
        () -> NOT_NULL_NO_NULLS_WITH_COLLECTION_VIOLATION);
  }

  /**
   * Assert that the supplied {@link Map} and none of its values are {@code null}.
   *
   * @param map the map to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied map as a convenience
   * @throws PreconditionViolationException if the supplied map or any of its values is {@code
   * null}
   * @see Condition#notNull(Object)
   * @see Condition#containsNoNulls(Map)
   * @see #requires(boolean, boolean, Supplier)
   */
  public static <T extends Map<?, ?>> T requiresNotNullNoNulls(final T map,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notNull(map), containsNoNulls(map), messageSupplier);
    return map;
  }

  /**
   * @see #requiresNotNullNoNulls(Map, Supplier)
   */
  public static <T extends Map<?, ?>> T requiresNotNullNoNulls(final T map)
      throws PreconditionViolationException {
    return requiresNotNullNoNulls(map, () -> NOT_NULL_NO_NULLS_WITH_MAP_VIOLATION);
  }

  /**
   * Assert that the supplied array and none of the objects it contains are
   * {@code null} and that there is at least one element.
   *
   * @param objects the array to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied collection as a convenience
   * @throws PreconditionViolationException if the supplied array is {@code null} or empty
   * @see Condition#notEmpty(Object[])
   * @see Condition#containsNoNulls(Object[])
   * @see #requires(boolean, boolean, Supplier)
   */
  public static Object[] requiresNotEmptyNoNulls(final Object[] objects,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notEmpty(objects), containsNoNulls(objects), messageSupplier);
    return objects;
  }

  /**
   * @see #requiresNotEmptyNoNulls(Object[], Supplier)
   */
  public static Object[] requiresNotEmptyNoNulls(final Object[] objects)
      throws PreconditionViolationException {
    return requiresNotEmptyNoNulls(objects, () -> NOT_EMPTY_NO_NULLS_WITH_ARRAY_VIOLATION);
  }

  /**
   * Assert that the supplied {@link Collection} is not {@code null}, containing no {@code null}s,
   * nor is empty (i.e. length == 0).
   *
   * @param collection the collection to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied collection as a convenience
   * @throws PreconditionViolationException if the supplied collection is {@code null}, any of its
   * elements is {@code null}, or is empty (i.e. length == 0)
   * @see Condition#notEmpty(Collection)
   * @see Condition#containsNoNulls(Collection)
   * @see #requires(boolean, boolean, Supplier)
   */
  public static <T extends Collection<?>> T requiresNotEmptyNoNulls(final T collection,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notEmpty(collection), containsNoNulls(collection), messageSupplier);
    return collection;
  }

  /**
   * @see #requiresNotEmptyNoNulls(Collection, Supplier)
   */
  public static <T extends Collection<?>> T requiresNotEmptyNoNulls(final T collection)
      throws PreconditionViolationException {
    return requiresNotEmptyNoNulls(collection,
        () -> NOT_EMPTY_NO_NULLS_WITH_COLLECTION_VIOLATION);
  }

  /**
   * Assert that the supplied {@link Map} is not {@code null}, containing no {@code null} values,
   * nor is empty (i.e. length == 0).
   *
   * @param map the collection to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied collection as a convenience
   * @throws PreconditionViolationException if the supplied collection is {@code null}, any of its
   * elements is {@code null}, or empty (i.e. length == 0)
   * @see Condition#notEmpty(Map)
   * @see Condition#containsNoNulls(Map)
   * @see #requires(boolean, boolean, Supplier)
   */
  public static <T extends Map<?, ?>> T requiresNotEmptyNoNulls(final T map,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notEmpty(map), containsNoNulls(map), messageSupplier);
    return map;
  }

  /**
   * @see #requiresNotEmptyNoNulls(Map, Supplier)
   */
  public static <T extends Map<?, ?>> T requiresNotEmptyNoNulls(final T map)
      throws PreconditionViolationException {
    return requiresNotEmptyNoNulls(map, () -> NOT_EMPTY_NO_NULLS_WITH_MAP_VIOLATION);
  }

  /**
   * Assert that the supplied {@link Number} is not negative; i.e. assert that {@link Number} is
   * >= 0.
   *
   * @param number the number to check
   * @param messageSupplier precondition violation message supplier
   * @return the supplied number as a convenience
   * @throws PreconditionViolationException if the supplied number is < 0
   * @see Condition#notNegative(Number)
   * @see #requires(boolean, Supplier)
   */
  public static <T extends Number> T requiresNotNegative(final T number,
      final Supplier<String> messageSupplier)
      throws PreconditionViolationException {
    requires(notNegative(number), messageSupplier);
    return number;
  }

  /**
   * @see #requiresNotNegative(Number, Supplier)
   */
  public static <T extends Number> T requiresNotNegative(final T number)
      throws PreconditionViolationException {
    return requiresNotNegative(number, () -> NOT_NEGATIVE_NUMBER_VIOLATION);
  }

  private static final String NOT_NULL_VIOLATION = "Requires argument to be not null.";
  private static final String NOT_NULL_NO_NULLS_WITH_ARRAY_VIOLATION = "Requires array to be not null nor contain any nulls.";
  private static final String NOT_NULL_NO_NULLS_WITH_COLLECTION_VIOLATION = "Requires collection to be not null nor contain any null entries.";
  private static final String NOT_NULL_NO_NULLS_WITH_MAP_VIOLATION = "Requires map to be not null nor contain any null values.";
  private static final String NOT_EMPTY_NO_NULLS_WITH_ARRAY_VIOLATION = "Requires array to be not be null, empty, nor contain any nulls.";
  private static final String NOT_EMPTY_NO_NULLS_WITH_COLLECTION_VIOLATION = "Requires collection to be not be null, empty, nor contain any null entries.";
  private static final String NOT_EMPTY_NO_NULLS_WITH_MAP_VIOLATION = "Requires map to be not be null, empty, nor contain any null values.";
  private static final String NOT_NEGATIVE_NUMBER_VIOLATION = "Requires number to be greater than or equal to zero.";

  /**
   * The exception type for a {@link Precondition} violation.
   */
  public static final class PreconditionViolationException extends IllegalArgumentException {

    private PreconditionViolationException(final String message) {
      super(message);
    }
  }

  /**
   * Supplemental class for conditions that can be used in contracts.
   *
   * @see Precondition
   */
  public static final class Condition {

    /**
     * Checks whether the given {@code object} is not {@code null}.
     *
     * @param object the parameter to check
     * @return true iff {@code object} is not {@code null}
     */
    public static boolean notNull(final Object object) {
      return null != object;
    }



    /**
     * Checks whether the given array of {@code objects} is not empty (length > 0).
     * <p>
     * Note that {@code null}, logically, has no length, and thus, does not fulfill this
     * condition.
     *
     * @param objects the parameter to check
     * @return true iff {@code objects} is not null nor empty (length > 0)
     */
    public static boolean notEmpty(final Object[] objects) {
      return null != objects && objects.length > 0;
    }

    /**
     * Checks whether the given {@code collection} is not empty (length > 0).
     * <p>
     * Note that {@code null}, logically, has no length, and thus, does not fulfill this
     * condition.
     *
     * @param collection the parameter to check
     * @return true iff {@code collection} is not null nor empty (length > 0)
     */
    public static boolean notEmpty(final Collection<?> collection) {
      return null != collection && !collection.isEmpty();
    }

    /**
     * Checks whether the given {@code map} is not empty (length > 0).
     * <p>
     * Note that {@code null}, logically, has no length, and thus, does not fulfill this
     * condition.
     *
     * @param map the parameter to check
     * @return true iff {@code map} is not null nor empty (length > 0)
     */
    public static boolean notEmpty(final Map<?, ?> map) {
      return null != map && !map.isEmpty();
    }

    /**
     * Checks whether the given array of {@code objects} contains no {@code null}s.
     * <p>
     * Note that {@code null}, logically, contains no {@code null}s.
     *
     * @param objects the parameter to check
     * @return true iff {@code objects} comprises no {@code null}s (or is {@code null})
     */
    public static boolean containsNoNulls(final Object[] objects) {
      return null == objects || Arrays.stream(objects).allMatch(Objects::nonNull);
    }

    /**
     * Checks whether the given {@code collection} contains no {@code null}s.
     * <p>
     * Note that {@code null}, logically, contains no {@code null}s.
     *
     * @param collection the parameter to check
     * @return true iff {@code collection} comprises no {@code null}s (or is {@code null})
     */
    public static boolean containsNoNulls(final Collection<?> collection) {
      return null == collection || collection.stream().allMatch(Objects::nonNull);
    }

    /**
     * Checks whether the given {@code map} contains no {@code null} values.
     * <p>
     * Note that {@code null}, logically, contains no {@code null}s.
     *
     * @param map the parameter to check
     * @return true iff {@code map} comprises no {@code null} values (or is {@code null})
     */
    public static boolean containsNoNulls(final Map<?, ?> map) {
      return null == map || map.values().stream().allMatch(Objects::nonNull);
    }

    /**
     * Checks whether the given {@code number} is not negative; i.e., must be >= 0.
     * <p>
     * Note that {@code null}, logically, is not >= 0.
     *
     * @param number the parameter to check
     * @return true iff {@code number} is not {@code null} and < 0
     */
    public static boolean notNegative(final Number number) {
      return null != number && 0 <= Double.compare(number.doubleValue(), 0);
    }
  }
}
