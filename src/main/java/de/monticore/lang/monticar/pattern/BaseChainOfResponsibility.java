package de.monticore.lang.monticar.pattern;

/**
 * Implements the chain-of-responsibility pattern
 *
 * @author Christoph Richter
 */
public abstract class BaseChainOfResponsibility<T> {

    private T successor = null;

    public T getSuccessor() {
        return successor;
    }

    public void setSuccessor(T successor) {
        this.successor = successor;
    }

    public abstract String getRole();

}
