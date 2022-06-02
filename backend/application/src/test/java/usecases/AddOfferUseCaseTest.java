package usecases;

import commands.AddOfferCommand;
import entity.DataRow;
import entity.Metadata;
import entity.Offer;
import entity.Policy;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.MockitoAnnotations;
import org.springframework.boot.test.mock.mockito.MockBean;
import ports.OfferPersistencePort;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;


class AddOfferUseCaseTest {

	@InjectMocks
	private AddOfferUseCase addOfferUseCase;

	@MockBean
	private OfferPersistencePort persistencePort;

	@BeforeEach
	public void init() {
		MockitoAnnotations.initMocks(this);
	}

	@Test
	void handle() {
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

		AddOfferCommand command = new AddOfferCommand(metadata, List.of(dataRow));

		Offer offer = addOfferUseCase.handle(command);

		assertAll(
				() -> assertEquals(metadata, offer.getMetadata()),
				() -> assertEquals(List.of(dataRow), offer.getData())
		);
	}
}
