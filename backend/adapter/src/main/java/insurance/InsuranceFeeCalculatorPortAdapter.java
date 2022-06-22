package insurance;

import entity.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;
import ports.InsuranceFeeCalculatorPort;

@AllArgsConstructor
public class InsuranceFeeCalculatorPortAdapter implements InsuranceFeeCalculatorPort {

	private final FeeCalculator calculator;

	@Override
	public double calculateFee(Dataset dataset) {
		return calculator.calculate(dataset.getData());
	}
}
