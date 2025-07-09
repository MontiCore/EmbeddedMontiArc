package insurance;

import entity.DataRow;

import java.util.List;

public interface FeeCalculator {
	double calculate(List<DataRow> dataRows);
}
