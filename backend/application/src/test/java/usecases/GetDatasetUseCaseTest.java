package usecases;

import entity.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.DatasetPersistencePort;
import ports.DsEnforcementPort;
import queries.GetDatasetQuery;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class GetDatasetUseCaseTest {

	@InjectMocks
	private GetDatasetUseCase underTest;

	@Mock
	private DatasetPersistencePort datasetPersistencePort;

	@Mock
	private DsEnforcementPort<Dataset> enforcementPort;

	@Test
	void shouldReturnDataset() {
		// given
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
		given(enforcementPort.enforce(any())).willReturn(dataset);

		// when
		ArgumentCaptor<Dataset> datasetArgumentCaptor =
				ArgumentCaptor.forClass(Dataset.class);

		Dataset actualDataset = underTest.handle(new GetDatasetQuery(uuid));

		// then
		verify(enforcementPort).enforce(datasetArgumentCaptor.capture());
		Dataset capturedDataset = datasetArgumentCaptor.getValue();
		assertThat(actualDataset.getId()).isEqualTo(uuid);
		assertThat(actualDataset).isEqualTo(capturedDataset);
	}
}
