package usecases;

import commands.DeleteDatasetCommand;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.DatasetPersistencePort;

import java.util.UUID;

import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class DeleteDatasetUseCaseTest {

	@InjectMocks
	private DeleteDatasetUseCase underTest;

	@Mock
	private DatasetPersistencePort persistencePort;

	@Test
	void shouldDeleteCorrectDataset() {
		underTest.handle(new DeleteDatasetCommand(UUID.fromString("f3875794-e220-44c5-bde1-e328505387e6")));

		verify(persistencePort).deleteById(UUID.fromString("f3875794-e220-44c5-bde1-e328505387e6"));
	}
}
