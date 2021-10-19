package de.rwth.montisim.commons.boundingbox;

public class UnimplementedCollisionException extends IllegalArgumentException {
    public UnimplementedCollisionException(String combination) {
        super(combination+" collision not supported. (AABB is only implemented for high-level AABB-AABB checks)");
    }
}
