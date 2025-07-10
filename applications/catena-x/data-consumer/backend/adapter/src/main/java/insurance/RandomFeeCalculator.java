package insurance;

import entity.DataRow;
import org.springframework.stereotype.Component;

import java.util.List;
import java.util.Random;

/**
 * This adapter uses a Random number generator. It returns values between
 * 1000 and 6000, which are realistic insurance fees in 2022 for trucks with
 * a weight over 3,5 t in freight transportation according to:
 * https://www.versicherungscheck.net/lkw-versicherung-kosten/
 * In real applications regression can be used to calculate the insurance fee.
 */
@Component
public class RandomFeeCalculator implements FeeCalculator {

	private final double MIN_FEE = 1000;
	private final double MAX_FEE = 6000;

	@Override
	public double calculate(List<DataRow> dataRows) {
		return (new Random().nextDouble() * (MAX_FEE - MIN_FEE)) + MIN_FEE;
	}
}
