package ports;

import entity.Dataset;

public interface InsuranceFeeCalculatorPort {
	double calculateFee(Dataset dataset);
}
