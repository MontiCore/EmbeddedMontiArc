package schemalang.validation.model;

import java.math.BigDecimal;

public class Range {

    private BigDecimal startValue;
    private BigDecimal endValue;

    public BigDecimal getStartValue() {
        return startValue;
    }

    public void setStartValue(BigDecimal startValue) {
        this.startValue = startValue;
    }

    public BigDecimal getEndValue() {
        return endValue;
    }

    public void setEndValue(BigDecimal endValue) {
        this.endValue = endValue;
    }
}