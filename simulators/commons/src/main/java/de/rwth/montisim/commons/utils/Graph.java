/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Vector;

public class Graph {
    public final Vector<List<Integer>> adjacencies;

    public Graph(int vertexCount){
        adjacencies = new Vector<>(vertexCount);
        adjacencies.setSize(vertexCount);
        for (int i = 0; i < vertexCount; ++i){
            adjacencies.set(i, new ArrayList<>());
        }
    }

    public void addUndirectedEdge(int v1, int v2){
        adjacencies.elementAt(v1).add(v2);
        adjacencies.elementAt(v2).add(v1);
    }

    public <S> Vector<S> newColor(S value){
        Vector<S> color = new Vector<>();
        color.setSize(adjacencies.size());
        for (int i = 0; i < color.size(); ++i){
            color.set(i, value);
        }
        return color;
    }

    public Deque<Integer> newStack(){
        return new ArrayDeque<>(adjacencies.size());
    }
}