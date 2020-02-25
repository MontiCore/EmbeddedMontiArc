/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.composite;

/**
 * BusValue objects should only content basic data types (double, int, ...).
 * Complex data types should be represented by BusComposite or BusArray.
 */
public class BusValue implements BusComponent {
    private Object value;

    public BusValue(Object value) {
        this.value = value;
    }

    public void setValue(Object value) {
        this.value = value;
    }

    public Object getValue() {
        return value;
    }

    public ComponentType getType() {
        return ComponentType.VALUE;
    }
}
