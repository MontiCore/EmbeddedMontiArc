/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.order;

/**
 * Based on the ExecutionOrder file of the montiarc-executionorder project.
 * It has to be included here, as another EMAComponentInstanceSymbol(from embedded-montiarc) is used.
 * Which does not extends the montiarc-executionorder EMAComponentInstanceSymbol.
 * Also using this as a possibility to provide an implementation for an embedded-montiarc-math generator
 * that can be reused while only the target language implementation has to change.
 *
 */
public interface ExecutionOrder extends Comparable {

    String INTERFACE_NAME = "ExecutionOrder";

    String toString();

    int getS();

    int getB();

    boolean equals(Object obj);

    int hashCode();

    int compareTo(Object obj);

}
