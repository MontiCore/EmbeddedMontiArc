package usecases;

import dto.DatasetView;
import entity.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.DatasetPersistencePort;
import ports.InsuranceFeeCalculatorPort;
import ports.PolicyEnforcementPort;
import queries.GetDatasetQuery;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class GetDatasetViewUseCaseTest {

	@InjectMocks
	private GetDatasetViewUseCase underTest;

	@Mock
	private DatasetPersistencePort datasetPersistencePort;

	@Mock
	private InsuranceFeeCalculatorPort feeCalculatorPort;

	@Mock
	private PolicyEnforcementPort<Dataset> enforcementPort;

	@Captor
	private ArgumentCaptor<Dataset> datasetArgumentCaptor;

	@Test
	void shouldReturnCorrectDatasetView() {
		UUID uuid = UUID.fromString("84c7f4ef-17d3-49de-b757-93b9db2ea9e8");

		Policy policy = new Policy();
		policy.setId(9);
		policy.setStartTime(LocalTime.of(8, 0));

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
				LocalDateTime.now(),
				42,
				50,
				22000,
				50,
				LocalDateTime.now()
		);

		Dataset dataset = new Dataset(
				uuid,
				new Offer(UUID.randomUUID(),
						metadata,
						List.of(dataRow)
				),
				metadata,
				List.of(dataRow),
				LocalDateTime.now()
		);

		given(datasetPersistencePort.findById(any())).willReturn(dataset);
		given(enforcementPort.enforcePolicy(any())).willReturn(dataset);
		given(feeCalculatorPort.calculateFee(any())).willReturn(2000.0);

		DatasetView datasetView = underTest.handle(new GetDatasetQuery(uuid));

		verify(enforcementPort).enforcePolicy(datasetArgumentCaptor.capture());
		Dataset capturedDataset = datasetArgumentCaptor.getValue();
		assertThat(datasetView.getId()).isEqualTo(uuid);
		assertThat(datasetView.getId()).isEqualTo(capturedDataset.getId());
		assertThat(datasetView.getNumberOfTrucks()).isEqualTo(1);
		assertThat(datasetView.getDrivingTrucks()).isEqualTo(1);
		assertThat(datasetView.getRestingTrucks()).isEqualTo(0);
		assertThat(datasetView.getAvgFuelConsumption()).isEqualTo(50);
		assertThat(datasetView.getInsuranceFee()).isEqualTo(2000.0);
	}
}
