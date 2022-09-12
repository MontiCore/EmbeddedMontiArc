/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.streamunits._symboltable;

import java.util.ArrayList;
import java.util.List;

/**
 *         This class stores the rows of a stream value matrix/array
 */
public class StreamValues {

    protected List<List<List<IStreamValue>>> streamValues = new ArrayList<>();

    public StreamValues() {

    }

    public StreamValues(List<IStreamValue> streamValues) {
        List<List<IStreamValue>> matrix = new ArrayList<>();
        matrix.add(streamValues);
        this.streamValues.add(matrix);
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

//    public void setStreamValues(List<List<List<IStreamValue>>> streamValues) {
//        this.streamValues = streamValues;
//    }

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
