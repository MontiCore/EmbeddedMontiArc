/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.contract;

import static de.monticore.lang.monticar.contract.Precondition.requires;
import static de.monticore.lang.monticar.contract.Precondition.requiresNotEmptyNoNulls;
import static de.monticore.lang.monticar.contract.Precondition.requiresNotNegative;
import static de.monticore.lang.monticar.contract.Precondition.requiresNotNull;
import static de.monticore.lang.monticar.contract.Precondition.requiresNotNullNoNulls;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatExceptionOfType;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

@DisplayName("Precondition")
class PreconditionTest {

  private static final Class<Precondition.PreconditionViolationException> VIOLATION_EXCEPTION_TYPE
      = Precondition.PreconditionViolationException.class;
  private static final String VIOLATION_MESSAGE = "VIOLATION MESSAGE";

  private static final boolean PREDICATE_FALSE = false;
  private static final boolean PREDICATE_TRUE = true;

  private static final Object NULL_OBJECT = null;
  private static final Object ANY_OBJECT = new Object();

  private static final Object[] NULL_ARRAY = null;
  private static final Object[] EMPTY_ARRAY = {};
  private static final Object[] ARRAY_WITH_NULLS = {ANY_OBJECT, null, ANY_OBJECT};
  private static final Object[] NON_EMPTY_ARRAY_WITHOUT_NULLS = {ANY_OBJECT, ANY_OBJECT};

  private static final Collection<?> NULL_COLLECTION = null;
  private static final Collection<?> EMPTY_COLLECTION = Collections.emptyList();
  private static final Collection<?> COLLECTION_WITH_NULLS = collectionFromArray(ARRAY_WITH_NULLS);
  private static final Collection<?> NON_EMPTY_COLLECTION_WITHOUT_NULLS = collectionFromArray(
      NON_EMPTY_ARRAY_WITHOUT_NULLS);

  private static final Map<?, ?> NULL_MAP = null;
  private static final Map<?, ?> EMPTY_MAP = Collections.emptyMap();
  private static final Map<?, ?> MAP_WITH_NULLS = mapFromCollection(COLLECTION_WITH_NULLS);
  private static final Map<?, ?> NON_EMPTY_MAP_WITHOUT_NULLS = mapFromCollection(
      NON_EMPTY_COLLECTION_WITHOUT_NULLS);

  private static final Number NULL_NUMBER = null;
  private static final Number NUMBER_ZERO = 0;
  private static final Number NUMBER_ZERO_PLUS_EPSILON = Double.MIN_VALUE;
  private static final Number NUMBER_ZERO_MINUS_EPSILON = -Double.MIN_VALUE;
  private static final Number POSITIVE_NUMBER = 1;
  private static final Number NEGATIVE_NUMBER = -1;
  private static final Number LARGE_POSITIVE_NUMBER = Double.MAX_VALUE;
  private static final Number LARGE_NEGATIVE_NUMBER = -Double.MAX_VALUE;

  @Nested
  class Requires {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenPredicate1IsFalse() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requires(PREDICATE_FALSE));
      }

      @Test
      void whenPredicate2IsFalse() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requires(PREDICATE_TRUE, PREDICATE_FALSE));
      }

      @Test
      void whenPredicate3IsFalse() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requires(PREDICATE_TRUE,
                PREDICATE_TRUE,
                PREDICATE_FALSE));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenPredicate1IsFalse() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requires(PREDICATE_FALSE, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenPredicate2IsFalse() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requires(PREDICATE_TRUE,
                  PREDICATE_FALSE,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenPredicate3IsFalse() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requires(PREDICATE_TRUE,
                  PREDICATE_TRUE,
                  PREDICATE_FALSE,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldNotThrowViolationException {

      @Test
      void whenPredicateIsTrue() {
        requires(PREDICATE_TRUE);
      }

      @Test
      void whenBothPredicatesAreTrue() {
        requires(PREDICATE_TRUE, PREDICATE_TRUE);
      }

      @Test
      void whenAllThreePredicatesAreTrue() {
        requires(PREDICATE_TRUE, PREDICATE_TRUE, PREDICATE_TRUE);
      }

      @Nested
      class WithMessageIgnored {

        @Test
        void whenPredicateIsTrue() {
          requires(PREDICATE_TRUE, () -> VIOLATION_MESSAGE);
        }

        @Test
        void whenBothPredicatesAreTrue() {
          requires(PREDICATE_TRUE, PREDICATE_TRUE, () -> VIOLATION_MESSAGE);
        }

        @Test
        void whenAllThreePredicatesAreTrue() {
          requires(PREDICATE_TRUE,
              PREDICATE_TRUE,
              PREDICATE_TRUE,
              () -> VIOLATION_MESSAGE);
        }
      }
    }
  }

  @Nested
  class RequiresNotNull {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullObjectIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNull(NULL_OBJECT));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullObjectIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNull(NULL_OBJECT, () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenNonNullObjectIsGiven() {
        assertThat(requiresNotNull(ANY_OBJECT)).isEqualTo(ANY_OBJECT);
      }
    }
  }

  @Nested
  class RequiresNotNullNoNulls {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullArrayIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNullNoNulls(NULL_ARRAY));
      }

      @Test
      void whenArrayWithNullsIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNullNoNulls(ARRAY_WITH_NULLS));
      }

      @Test
      void whenNullCollectionIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNullNoNulls(NULL_COLLECTION));
      }

      @Test
      void whenCollectionWithNullsIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNullNoNulls(COLLECTION_WITH_NULLS));
      }

      @Test
      void whenNullMapIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNullNoNulls(NULL_MAP));
      }

      @Test
      void whenMapWithNullsIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNullNoNulls(MAP_WITH_NULLS));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullArrayIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNullNoNulls(NULL_ARRAY,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenArrayWithNullsIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNullNoNulls(ARRAY_WITH_NULLS,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNullCollectionIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNullNoNulls(NULL_COLLECTION,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenCollectionWithNullsIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNullNoNulls(COLLECTION_WITH_NULLS,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNullMapIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNullNoNulls(NULL_MAP,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenMapWithNullsIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNullNoNulls(MAP_WITH_NULLS,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenEmptyArrayIsGiven() {
        assertThat(requiresNotNullNoNulls(EMPTY_ARRAY)).isEqualTo(EMPTY_ARRAY);
      }

      @Test
      void whenNonEmptyArrayWithoutNullsIsGiven() {
        assertThat(requiresNotNullNoNulls(NON_EMPTY_ARRAY_WITHOUT_NULLS)).isEqualTo(
            NON_EMPTY_ARRAY_WITHOUT_NULLS);
      }

      @Test
      void whenEmptyCollectionIsGiven() {
        assertThat(requiresNotNullNoNulls(EMPTY_COLLECTION)).isEqualTo(EMPTY_COLLECTION);
      }

      @Test
      void whenNonEmptyCollectionWithoutNullsIsGiven() {
        assertThat(requiresNotNullNoNulls(NON_EMPTY_COLLECTION_WITHOUT_NULLS)).isEqualTo(
            NON_EMPTY_COLLECTION_WITHOUT_NULLS);
      }

      @Test
      void whenEmptyMapIsGiven() {
        assertThat(requiresNotNullNoNulls(EMPTY_MAP)).isEqualTo(EMPTY_MAP);
      }

      @Test
      void whenNonEmptyMapWithoutNullsIsGiven() {
        assertThat(requiresNotNullNoNulls(NON_EMPTY_MAP_WITHOUT_NULLS)).isEqualTo(
            NON_EMPTY_MAP_WITHOUT_NULLS);
      }
    }
  }

  @Nested
  class RequiresNotEmptyNoNulls {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullArrayIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(NULL_ARRAY));
      }

      @Test
      void whenArrayWithNullsIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(ARRAY_WITH_NULLS));
      }

      @Test
      void whenEmptyArrayIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(EMPTY_ARRAY));
      }

      @Test
      void whenNullCollectionIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(NULL_COLLECTION));
      }

      @Test
      void whenCollectionWithNullsIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(COLLECTION_WITH_NULLS));
      }

      @Test
      void whenEmptyCollectionIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(EMPTY_COLLECTION));
      }

      @Test
      void whenNullMapIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(NULL_MAP));
      }

      @Test
      void whenMapWithNullsIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(MAP_WITH_NULLS));
      }

      @Test
      void whenEmptyMapIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotEmptyNoNulls(EMPTY_MAP));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullArrayIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(NULL_ARRAY,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenArrayWithNullsIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(ARRAY_WITH_NULLS,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenEmptyArrayIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(EMPTY_ARRAY,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNullCollectionIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(NULL_COLLECTION,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenCollectionWithNullsIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(COLLECTION_WITH_NULLS,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenEmptyCollectionIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(EMPTY_COLLECTION,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenNullMapIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(NULL_MAP,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenMapWithNullsIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(MAP_WITH_NULLS,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenEmptyMapIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotEmptyNoNulls(EMPTY_MAP,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenNonEmptyArrayWithoutNullsIsGiven() {
        assertThat(requiresNotEmptyNoNulls(NON_EMPTY_ARRAY_WITHOUT_NULLS)).isEqualTo(
            NON_EMPTY_ARRAY_WITHOUT_NULLS);
      }

      @Test
      void whenNonEmptyCollectionWithoutNullsIsGiven() {
        assertThat(requiresNotEmptyNoNulls(NON_EMPTY_COLLECTION_WITHOUT_NULLS)).isEqualTo(
            NON_EMPTY_COLLECTION_WITHOUT_NULLS);
      }

      @Test
      void whenNonEmptyMapWithoutNullsIsGiven() {
        assertThat(requiresNotEmptyNoNulls(NON_EMPTY_MAP_WITHOUT_NULLS)).isEqualTo(
            NON_EMPTY_MAP_WITHOUT_NULLS);
      }
    }
  }



  @Nested
  class RequiresNotNegative {

    @Nested
    class ShouldThrowViolationException {

      @Test
      void whenNullNumberIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNegative(NULL_NUMBER));
      }

      @Test
      void whenSlightlySmallerNumberThenZeroIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNegative(NUMBER_ZERO_MINUS_EPSILON));
      }

      @Test
      void whenANegativeNumberIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNegative(NEGATIVE_NUMBER));
      }

      @Test
      void whenALargeNegativeNumberIsGiven() {
        assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
            .isThrownBy(() -> requiresNotNegative(LARGE_NEGATIVE_NUMBER));
      }

      @Nested
      class WithExpectedMessage {

        @Test
        void whenNullNumberIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNegative(NULL_NUMBER,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenSlightlySmallerNumberThenZeroIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNegative(NUMBER_ZERO_MINUS_EPSILON,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenANegativeNumberIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNegative(NEGATIVE_NUMBER,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }

        @Test
        void whenALargeNegativeNumberIsGiven() {
          assertThatExceptionOfType(VIOLATION_EXCEPTION_TYPE)
              .isThrownBy(() -> requiresNotNegative(LARGE_NEGATIVE_NUMBER,
                  () -> VIOLATION_MESSAGE))
              .withMessage(VIOLATION_MESSAGE);
        }
      }
    }

    @Nested
    class ShouldReturnParameterAsConvenience {

      @Test
      void whenNumberZeroIsGiven() {
        assertThat(requiresNotNegative(NUMBER_ZERO)).isEqualTo(NUMBER_ZERO);
      }

      @Test
      void whenSlightlyLargerNumberThenZeroIsGiven() {
        assertThat(requiresNotNegative(NUMBER_ZERO_PLUS_EPSILON)).isEqualTo(
            NUMBER_ZERO_PLUS_EPSILON);
      }

      @Test
      void whenAPositiveNumberIsGiven() {
        assertThat(requiresNotNegative(POSITIVE_NUMBER)).isEqualTo(POSITIVE_NUMBER);
      }

      @Test
      void whenALargePositiveNumberIsGiven() {
        assertThat(requiresNotNegative(LARGE_POSITIVE_NUMBER)).isEqualTo(
            LARGE_POSITIVE_NUMBER);
      }
    }
  }

  private static Collection<Object> collectionFromArray(final Object[] array) {
    return Collections.unmodifiableCollection(Arrays.asList(array));
  }

  private static Map<?, ?> mapFromCollection(final Collection<?> collection) {
    final Map<Object, Object> mapFromCollection = new HashMap<Object, Object>(collection.size());
    collection.forEach(v -> mapFromCollection.put(new Object(), v));
    return Collections.unmodifiableMap(mapFromCollection);
  }
}
