/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

public class Pair<K, V> {
    private K key;
    private V value;

    public Pair(K key, V value) {
        this.key = key;
        this.value = value;
    }

    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (this == obj)
            return true;
        if (!(obj instanceof Pair<?, ?>))
            return false;
        Pair<?, ?> other = (Pair<?, ?>) obj;
        return key.equals(other.key) && value.equals(other.value);
    }

    public K getKey() {
        return key;
    }

    public V getValue() {
        return value;
    }

    public int hashCode() {
        return key.hashCode() ^ value.hashCode();
    }

    public String toString() {
        return "<" + key + ", " + value + ">";
    }
}
