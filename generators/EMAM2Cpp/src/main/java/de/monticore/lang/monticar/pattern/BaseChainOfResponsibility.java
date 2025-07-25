/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.pattern;

/**
 * Implements the chain-of-responsibility pattern
 *
 */
public abstract class BaseChainOfResponsibility<T> {

    private BaseChainOfResponsibility<T> predecessor = null;

    private BaseChainOfResponsibility<T> successor = null;

    public BaseChainOfResponsibility<T> getSuccessor() {
        return successor;
    }

    protected BaseChainOfResponsibility<T> getPredecessor() {
        return predecessor;
    }

    public void setSuccessor(BaseChainOfResponsibility<T> successor) {
        this.successor = successor;
        successor.predecessor = this;
    }

    protected BaseChainOfResponsibility<T> getChainStart() {
        if (predecessor == null)
            return this;
        else
            return predecessor.getChainStart();
    }

    public abstract String getRole();

}
