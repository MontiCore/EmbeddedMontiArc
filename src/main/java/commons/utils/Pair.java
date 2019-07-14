/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package commons.utils;

public class Pair<K,V> {
    private K key;
    private V value;
    public Pair(K key, V value){
        this.key = key;
        this.value = value;
    }

    public boolean equals(Object obj){
        if (obj == null) return false;
        if (this == obj) return true;
        if (!(obj instanceof Pair)) return false;
        Pair other = (Pair) obj;
        return key.equals(other.key) && value.equals(other.value);
    }

    public K getKey(){
        return key;
    }
    public V getValue(){
        return value;
    }

    public int hashCode(){
        return key.hashCode() ^ value.hashCode();
    }

    public String toString(){
        return "<" + key + ", " + value + ">";
    }
}