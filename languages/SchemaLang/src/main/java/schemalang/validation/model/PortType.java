package schemalang.validation.model;

import java.util.Optional;

public class PortType {

    private String typeIdentifier;
    private Dimension dimension;
    private Range range;

    public String getTypeIdentifier() {
        return typeIdentifier;
    }

    public void setTypeIdentifier(String typeIdentifier) {
        this.typeIdentifier = typeIdentifier;
    }

    public Optional<Dimension> getDimension() {
        return Optional.ofNullable(dimension);
    }

    public void setDimension(Dimension dimension) {
        this.dimension = dimension;
    }

    public Optional<Range> getRange() {
        return Optional.ofNullable(range);
    }

    public void setRange(Range range) {
        this.range = range;
    }
}