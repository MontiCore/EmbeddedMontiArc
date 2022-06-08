package usecases;

import entity.DataRow;
import entity.Metadata;
import entity.Offer;
import entity.Policy;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.OfferPersistencePort;
import queries.GetOffersMetadataQuery;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class GetOffersMetadataUseCaseTest {

	@InjectMocks
	private GetOffersMetadataUseCase underTest;

	@Mock
	private OfferPersistencePort offerPersistencePort;

	@Test
	void shouldReturnTwoOffersMetadata() {
		// given
		UUID firstUUID = UUID.fromString("298384ab-ee2e-48d3-9e26-8895643316cb");
		UUID secondUUID = UUID.fromString("95d0ee07-e868-48ec-8322-3d7c2cad4c70");

		Policy firstPolicy = new Policy();
		firstPolicy.setId(1);
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

		given(offerPersistencePort.findAll()).willReturn(
				List.of(
						new Offer(
								firstUUID,
								firstMetadata,
								List.of(dataRow)
						)
						,
						new Offer(
								secondUUID,
								secondMetadata,
								List.of(dataRow)
						)));

		// when
		Map<UUID, Metadata> actualMetadataMap = underTest.handle(new GetOffersMetadataQuery(false));

		// then
		verify(offerPersistencePort).findAll();
		assertThat(actualMetadataMap.containsKey(firstUUID)).isTrue();
		assertThat(actualMetadataMap.containsKey(secondUUID)).isTrue();
		assertThat(actualMetadataMap.get(firstUUID).getPolicy().getId()).isEqualTo(1);
		assertThat(actualMetadataMap.get(secondUUID).getPolicy().getId()).isEqualTo(2);
	}

	@Test
	void shouldBeEmptyIfNoOffersAtAll() {
		// given
		given(offerPersistencePort.findAll()).willReturn(Collections.emptyList());

		// when
		Map<UUID, Metadata> actualMetadataMap = underTest.handle(new GetOffersMetadataQuery(false));

		// then
		verify(offerPersistencePort).findAll();
		assertThat(actualMetadataMap.size()).isEqualTo(0);
	}

	@Test
	void shouldReturnOneBoughtOffersMetadata() {
		// given
		UUID uuid = UUID.fromString("298384ab-ee2e-48d3-9e26-8895643316cb");

		Policy policy = new Policy();
		policy.setId(1);
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

		given(offerPersistencePort.findAllBought()).willReturn(
				List.of(
						new Offer(
								uuid,
								metadata,
								List.of(dataRow)
						)
				));

		// when
		Map<UUID, Metadata> actualMetadataMap = underTest.handle(new GetOffersMetadataQuery(true));

		// then
		verify(offerPersistencePort).findAllBought();
		assertThat(actualMetadataMap.containsKey(uuid)).isTrue();
		assertThat(actualMetadataMap.get(uuid).getPolicy().getId()).isEqualTo(1);
	}

	@Test
	void shouldBeEmptyIfNoOffersBought() {
		// given
		given(offerPersistencePort.findAllBought()).willReturn(Collections.emptyList());

		// when
		Map<UUID, Metadata> actualMetadataMap = underTest.handle(new GetOffersMetadataQuery(true));

		// then
		verify(offerPersistencePort).findAllBought();
		assertThat(actualMetadataMap.size()).isEqualTo(0);
	}

}
