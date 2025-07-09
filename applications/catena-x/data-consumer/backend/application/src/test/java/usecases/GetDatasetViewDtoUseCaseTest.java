package usecases;

import dto.DatasetViewDto;
import entity.DataRow;
import entity.Dataset;
import entity.Metadata;
import entity.Policy;
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
class GetDatasetViewDtoUseCaseTest {

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
				UUID.randomUUID(),
				metadata,
				List.of(dataRow),
				LocalDateTime.now()
		);

		given(datasetPersistencePort.findById(any())).willReturn(dataset);
		given(enforcementPort.enforcePolicy(any())).willReturn(dataset);
		given(feeCalculatorPort.calculateFee(any())).willReturn(2000.0);

		DatasetViewDto datasetViewDto = underTest.handle(new GetDatasetQuery(uuid));

		verify(enforcementPort).enforcePolicy(datasetArgumentCaptor.capture());
		Dataset capturedDataset = datasetArgumentCaptor.getValue();
		assertThat(datasetViewDto.getId()).isEqualTo(uuid);
		assertThat(datasetViewDto.getId()).isEqualTo(capturedDataset.getId());
		assertThat(datasetViewDto.getNumberOfTrucks()).isEqualTo(1);
		assertThat(datasetViewDto.getDrivingTrucks()).isEqualTo(1);
		assertThat(datasetViewDto.getRestingTrucks()).isEqualTo(0);
		assertThat(datasetViewDto.getAvgFuelConsumption()).isEqualTo(50);
		assertThat(datasetViewDto.getInsuranceFee()).isEqualTo(2000.0);
	}
}
