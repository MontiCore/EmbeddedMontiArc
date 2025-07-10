package usecases;

import commands.AddOfferCommand;
import entity.DataRow;
import entity.Metadata;
import entity.Offer;
import entity.Policy;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.OfferPersistencePort;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;

import static org.assertj.core.api.AssertionsForClassTypes.assertThat;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
public class AddOfferUseCaseTest {

	@InjectMocks
	private AddOfferUseCase underTest;

	@Mock
	private OfferPersistencePort persistencePort;

	@Captor
	private ArgumentCaptor<Offer> offerArgumentCaptor;

	@Test
	public void shouldSaveOffer() {
		Policy policy = new Policy();
		policy.setId(1);
		policy.setStartTime(LocalTime.of(8, 0));

		Metadata metadata = new Metadata(
				1,
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

		underTest.handle(new AddOfferCommand(metadata, List.of(dataRow)));

		verify(persistencePort).save(offerArgumentCaptor.capture());
		Offer capturedOffer = offerArgumentCaptor.getValue();
		assertThat(capturedOffer.getMetadata().getId()).isEqualTo(1);
	}
}
