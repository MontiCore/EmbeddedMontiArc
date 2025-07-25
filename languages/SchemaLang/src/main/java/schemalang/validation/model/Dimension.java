package schemalang.validation.model;

import com.google.common.collect.Lists;

import java.util.List;
import java.util.Objects;

public class Dimension {

    private int size;
    private List<Integer> dimensionList = Lists.newArrayList();

    public int getSize() {
        return size;
    }

    public void setSize(int size) {
        this.size = size;
    }

    public List<Integer> getDimensionList() {
        return dimensionList;
    }

    public void setDimensionList(List<Integer> dimensionList) {
        this.dimensionList = dimensionList;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Dimension dimension = (Dimension) o;
        return dimensionList.equals(dimension.dimensionList);
    }

    @Override
    public int hashCode() {
        return Objects.hash(dimensionList);
    }
}