package insurance;

import entity.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.List;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;

@ExtendWith(MockitoExtension.class)
class InsuranceFeeCalculatorPortAdapterTest {

	@InjectMocks
	private InsuranceFeeCalculatorPortAdapter underTest;

	@Mock
	private FeeCalculator calculator;

	@Test
	void calculateFee() {
		UUID uuid = UUID.fromString("d8e71f74-1c7e-4f69-a9db-786d65f7e17a");
		Policy policy = new Policy();
		policy.setMaxUsages(3);

		Metadata metadata = new Metadata(
				42,
				"Aachen Dataset",
				"Carrier GmbH",
				"A simple test data set...",
				10,
				"/logging",
				policy
		);

		DataRow dataRow = new DataRow(
				1,
				"1234",
				51,
				10,
				null,
				42,
				50,
				22000,
				50,
				null
		);
		Offer offer = new Offer(UUID.randomUUID(), metadata, List.of(dataRow));
		Dataset dataset = new Dataset(UUID.randomUUID(), offer.getId(), metadata, List.of(dataRow), null);
		given(calculator.calculate(any())).willReturn(2500.0);

		assertThat(underTest.calculateFee(dataset)).isEqualTo(2500);
	}
}
