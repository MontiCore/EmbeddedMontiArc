/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

/**
 */
public class StreamCompare {
    protected StreamValuePrecision left;
    protected StreamValuePrecision right;
    protected String operator;

    public StreamCompare(StreamValuePrecision left, String operator, StreamValuePrecision right) {
        this.left = left;
        this.operator = operator;
        this.right = right;
    }

    public StreamValuePrecision getLeft() {
        return left;
    }

    public void setLeft(StreamValuePrecision left) {
        this.left = left;
    }

    public StreamValuePrecision getRight() {
        return right;
    }

    public void setRight(StreamValuePrecision right) {
        this.right = right;
    }

    public String getOperator() {
        return operator;
    }

    public void setOperator(String operator) {
        this.operator = operator;
    }
}
