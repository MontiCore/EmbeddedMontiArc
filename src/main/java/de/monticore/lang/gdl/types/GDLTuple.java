package de.monticore.lang.gdl.types;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class GDLTuple implements GDLType {
    
    private List<GDLType> elements = new LinkedList<>();

    private GDLTuple() { }

    public GDLTuple(GDLType... elements) {
        this.elements = List.of(elements);
    }

    public GDLTuple(String... elements) {
        this.elements = Arrays.stream(elements)
                .map(e -> GDLType.createFromLine(e))
                .collect(Collectors.toList());
    }

    public GDLType get(int i) {
        return elements.get(i);
    }

    public int size() {
        return elements.size();
    }

    public Stream<GDLType> stream() {
        return elements.stream();
    }

    public static GDLTuple createFromLine(String line) {
        GDLTuple tuple = new GDLTuple();

        String stripped = line.strip();
        stripped = stripped.substring(1, stripped.length()-1);

        int para = 0;
        StringBuilder paraSegment = new StringBuilder();
        StringBuilder wordSegment = new StringBuilder();

        for (int i = 0; i < stripped.length(); i++) {
            char character = stripped.charAt(i);

            if (character == '(') {
                para++;
                paraSegment.append(character);
            } else if (character == ')') {
                para--;
                paraSegment.append(character);
                if (para == 0) {
                    tuple.elements.add(GDLType.createFromLine(paraSegment.toString()));
                    paraSegment = new StringBuilder();
                }
            } else if (character == ' ' && para == 0 && wordSegment.length() != 0) {
                tuple.elements.add(GDLType.createFromLine(wordSegment.toString()));
                wordSegment = new StringBuilder();
            } else if (para > 0) {
                paraSegment.append(character);
            } else if (character != ' ') {
                wordSegment.append(character);
            }
        }
        if (wordSegment.length() != 0) tuple.elements.add(GDLType.createFromLine(wordSegment.toString()));
        
        return tuple;
    }

    public static GDLTuple createFromPl(String plLine) {
        GDLTuple tuple = new GDLTuple();

        String stripped = plLine.strip();
        stripped = stripped.substring(1, stripped.length()-1);

        int para = 0;
        StringBuilder paraSegment = new StringBuilder();
        StringBuilder wordSegment = new StringBuilder();

        for (int i = 0; i < stripped.length(); i++) {
            char character = stripped.charAt(i);

            if (Character.isWhitespace(character)) {
                continue;
            }


            if (character == '[') {
                para++;
                paraSegment.append(character);
            } else if (character == ']') {
                para--;
                paraSegment.append(character);
                if (para == 0) {
                    tuple.elements.add(GDLType.createFromPl(paraSegment.toString()));
                    paraSegment = new StringBuilder();
                }
            } else if (character == ',' && para == 0 && wordSegment.length() != 0) {
                tuple.elements.add(GDLType.createFromPl(wordSegment.toString()));
                wordSegment = new StringBuilder();
            } else if (para > 0) {
                paraSegment.append(character);
            } else if (character != ',') {
                wordSegment.append(character);
            }
        }
        if (wordSegment.length() != 0) tuple.elements.add(GDLType.createFromPl(wordSegment.toString()));
        
        return tuple;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("(");
        for (int i = 0; i < elements.size(); i++) {
            GDLType element = elements.get(i);
            sb.append(element);

            if (i + 1 < elements.size()) sb.append(" ");
        }
        sb.append(")");
        return sb.toString();
    }

    @Override
    public String toPlString() {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < elements.size(); i++) {
            GDLType element = elements.get(i);
            sb.append(element.toPlString());

            if (i + 1 < elements.size()) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }

    @Override
    public int hashCode() {
        return elements.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof GDLTuple) {
            GDLTuple tuple = (GDLTuple) obj;
            return elements.equals(tuple.elements);
        }
        return super.equals(obj);
    }

}
