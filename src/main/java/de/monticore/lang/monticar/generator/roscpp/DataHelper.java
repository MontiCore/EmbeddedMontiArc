package de.monticore.lang.monticar.generator.roscpp;

public class DataHelper {
    private static ResolvedRosTag currentResolvedRosTag = new ResolvedRosTag(null);

    public static void setCurrentResolvedRosTag(ResolvedRosTag currentResolvedRosTag) {
        DataHelper.currentResolvedRosTag = currentResolvedRosTag;
    }

    public static ResolvedRosTag getResolvedRosTag() {
        return currentResolvedRosTag;
    }
}
