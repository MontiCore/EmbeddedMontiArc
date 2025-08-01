package de.monticore.lang.monticar.cnnarch.generator.training;

public enum NetworkType {

    GNN("gnn");

    String type;

    NetworkType(String type) {
        this.type = type;
    }

    public static NetworkType networkType(String type) {
        for (NetworkType nt : values()) {
            if (nt.type.equals(type)) {
                return nt;
            }
        }
        throw new IllegalArgumentException(String.valueOf(type));
    }

    public String getType() {
        return type;
    }
}