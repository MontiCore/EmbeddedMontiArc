package usecases;

import commands.DeleteOfferCommand;
import entity.Offer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;

import java.util.UUID;

import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class DeleteOfferUseCaseTest {

	@InjectMocks
	private DeleteOfferUseCase underTest;

	@Mock
	private DatasetPersistencePort datasetPersistencePort;

	@Mock
	private OfferPersistencePort offerPersistencePort;

	@Test
	void shouldOnlyDeleteOffer() {
		UUID uuid = UUID.fromString("f3875794-e220-44c5-bde1-e328505387e6");
		given(offerPersistencePort.findById(uuid)).willReturn(
				new Offer(
						uuid,
						null,
						null
				)
		);
		underTest.handle(new DeleteOfferCommand(uuid));

		verify(offerPersistencePort).deleteDatasetDataRowOfferById(uuid);
	}
}
