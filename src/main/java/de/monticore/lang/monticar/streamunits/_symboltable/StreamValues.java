/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 *         This class stores the rows of a stream value matrix/array
 */
public class StreamValues {

    protected List<List<List<IStreamValue>>> streamValues = new ArrayList<>();
    protected Optional<Double> elementTolerance = Optional.empty();
    protected Optional<Double> generalTolerance = Optional.empty();

    public StreamValues() {

    }

    public StreamValues(List<IStreamValue> streamValues) {
        List<List<IStreamValue>> matrix = new ArrayList<>();
        matrix.add(streamValues);
        this.streamValues.add(matrix);
    }

    public StreamValues(List<IStreamValue> streamValues, double elementTolerance, double generalTolerance) {
        List<List<IStreamValue>> matrix = new ArrayList<>();
        matrix.add(streamValues);
        this.streamValues.add(matrix);
        this.elementTolerance = Optional.of(elementTolerance);
        this.generalTolerance = Optional.of(generalTolerance);
    }

    public List<IStreamValue> getStreamValues(int rowIndex) {
        assert streamValues.size() == 1;

        return streamValues.get(0).get(rowIndex);
    }

    /**
     *
     * @param rowIndex starts at 0
     * @param columnIndex starts at 0
     * @return the element in row rowIndex and column columnIndex
     */
    public IStreamValue getStreamValue(int rowIndex, int columnIndex) {
        assert streamValues.size() == 1;

        return streamValues.get(0).get(rowIndex).get(columnIndex);
    }

    public IStreamValue getStreamValue(int depthIndex, int rowIndex, int columnIndex) {
        assert streamValues.size() > 0;
        return streamValues.get(depthIndex).get(rowIndex).get(columnIndex);
    }

    public void add(List<IStreamValue> streamValues) {
        if (this.streamValues.size() == 0) this.streamValues.add(new ArrayList<>());
        this.streamValues.get(0).add(streamValues);
    }

    public void setStreamValues(List<List<IStreamValue>> streamValues) {
        this.streamValues.add(streamValues);
    }

    public void setElementTolerance(double elementTolerance) { this.elementTolerance = Optional.of(elementTolerance);}

    public void setGeneralTolerance(double generalTolerance) { this.generalTolerance = Optional.of(generalTolerance);}

//    public void setStreamValues(List<List<List<IStreamValue>>> streamValues) {
//        this.streamValues = streamValues;
//    }
    public double getElementTolerance() {
        if (this.elementTolerance.isPresent()) {
            double tolVal = this.elementTolerance.get();
            if (tolVal > 1) {
                return 1;
            } else if (tolVal < 0) {
                return 0;
            } else {
                return tolVal;
            }
        } else {
            return 0;
        }
    }

    public double getGeneralTolerance() {
        if (this.generalTolerance.isPresent()) {
            double tolVal = this.generalTolerance.get();
            if (tolVal > 1) {
                return 1;
            } else if (tolVal < 0) {
                return 0;
            } else {
                return tolVal;
            }
        } else {
            return 0;
        }
    }

    public int getDepthDimension() {
        return streamValues.size();
    }

    /**
     * @return amount of rows
     */
    public int getRowDimension() {
        assert streamValues.size() > 0;

        return streamValues.get(0).size();
    }

    /**
     * @return amount of columns
     */
    public int getColumnDimension() {
        assert streamValues.size() > 0;
        if (streamValues.get(0).size() == 0) {
            return 0;
        }
        return streamValues.get(0).get(0).size();
    }

}
