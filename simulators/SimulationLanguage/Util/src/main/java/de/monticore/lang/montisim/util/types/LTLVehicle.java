/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.util.types;

import de.monticore.lang.montisim.carlang.CarContainer;

import java.util.List;
import java.util.Optional;

public class LTLVehicle {
    private List<double[]> path;
    private boolean isRandomPath;
    private CarContainer carContainer;

    private Optional<Double> startRotation;
    private Optional<String> networkType;
    private Optional<List<Goal>> goals;
    private Optional<Integer> platoonSize = Optional.empty();

    public LTLVehicle() {
    }

    public List<double[]> getPath() {
        return path;
    }

    public void setPath(List<double[]> path) {
        this.path = path;
    }

    public boolean isRandomPath() {
        return isRandomPath;
    }

    public void setRandomPath(boolean randomPath) {
        this.path = null;
        isRandomPath = randomPath;
    }

    public CarContainer getCarContainer() {
        return carContainer;
    }

    public Optional<Double> getStartRotation() {
        return startRotation;
    }

    public void setStartRotation(Optional<Double> startRotation) {
        this.startRotation = startRotation;
    }

    public Optional<String> getNetworkType() {
        return networkType;
    }

    public void setCarContainer(CarContainer carContainer) {
        this.carContainer = carContainer;
    }

    public void setNetworkType(Optional<String> networkType) {
        this.networkType = networkType;
    }

    public Optional<List<Goal>> getGoals() {
        return goals;
    }

    public void setGoals(Optional<List<Goal>> goals) {
        this.goals = goals;
    }

    public Optional<Integer> getPlatoonSize() {
        return platoonSize;
    }

    public void setPlatoonSize(Optional<Integer> platoonSize) {
        this.platoonSize = platoonSize;
    }

    public enum LTLOperator {
        ALWAYS,
        NEVER,
        EVENTUALLY,
        UNTIL
    }

    public enum Comparator {
        LESS_EQUAL,
        LESS,
        GREATER_EQUAL,
        GREATER,
        EQUAL
    }

    public static class Goal {
        public LTLOperator ltlOperator;
        public String metricName;
        public Comparator comparator;
        public NumberUnit targetMetric;

        public Goal(String ltlOperator, String metricName, String comparator, NumberUnit targetMetric) {

            // convert ltlOperator to enum
            for (LTLOperator type : LTLOperator.values()) {
                if (type.toString().equals(ltlOperator.toUpperCase())) {
                    this.ltlOperator = type;
                    break;
                }
            }

            // convert comparator to enum
            switch (comparator) {
                case "lt":
                    this.comparator = Comparator.LESS;
                    break;
                case "le":
                    this.comparator = Comparator.LESS_EQUAL;
                    break;
                case "gt":
                    this.comparator = Comparator.GREATER;
                    break;
                case "ge":
                    this.comparator = Comparator.GREATER_EQUAL;
                    break;
                case "eq":
                    this.comparator = Comparator.EQUAL;
                    break;
                default:
                    break;
            }

            this.metricName = metricName;
            this.targetMetric = targetMetric;
        }
    }
}
