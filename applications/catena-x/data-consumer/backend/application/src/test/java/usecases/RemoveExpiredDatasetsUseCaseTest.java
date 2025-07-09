package usecases;

import commands.RemoveExpiredDatasetsCommand;
import entity.DataRow;
import entity.Dataset;
import entity.Metadata;
import entity.Policy;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.DatasetPersistencePort;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;
import java.util.UUID;

import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class RemoveExpiredDatasetsUseCaseTest {

	@InjectMocks
	private RemoveExpiredDatasetsUseCase underTest;

	@Mock
	private DatasetPersistencePort persistencePort;

	@Test
	void shouldRemoveOneExpiredOutOfTwoDatasets() {
		UUID firstUUID = UUID.fromString("298384ab-ee2e-48d3-9e26-8895643316cb");
		UUID secondUUID = UUID.fromString("95d0ee07-e868-48ec-8322-3d7c2cad4c70");

		Policy firstPolicy = new Policy();
		firstPolicy.setId(1);
		firstPolicy.setExpiresOn(LocalDate.now().minusDays(1));
		firstPolicy.setStartTime(LocalTime.of(8, 0));

		Policy secondPolicy = new Policy();
		secondPolicy.setId(2);
		secondPolicy.setEndTime(LocalTime.of(17, 0));

		Metadata firstMetadata = new Metadata(
				42,
				"Aachen Dataset",
				"Carrier GmbH",
				"A simple test data set...",
				10,
				"/logging",
				firstPolicy
		);

		Metadata secondMetadata = new Metadata(
				93,
				"Friedrichshafen Dataset",
				"Dachser GmbH",
				"Another simple test data set...",
				20.55,
				"/logging",
				secondPolicy
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

		given(persistencePort.findAll()).willReturn(
				List.of(
						new Dataset(
								firstUUID,
								UUID.randomUUID(),
								firstMetadata,
								List.of(dataRow),
								LocalDateTime.now()
						), new Dataset(
								secondUUID,
								UUID.randomUUID(),
								secondMetadata,
								List.of(dataRow),
								LocalDateTime.now()
						)));

		underTest.handle(new RemoveExpiredDatasetsCommand());

		verify(persistencePort).deleteById(firstUUID);
		verify(persistencePort, never()).deleteById(secondUUID);
	}
}
