package usecases;

import commands.BuyOfferCommand;
import entity.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;
import ports.PolicyDeploymentPort;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;
import java.util.UUID;

import static org.assertj.core.api.AssertionsForClassTypes.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class BuyOfferUseCaseTest {

	@InjectMocks
	private BuyOfferUseCase underTest;

	@Mock
	private DatasetPersistencePort datasetPersistencePort;

	@Mock
	private OfferPersistencePort offerPersistencePort;

	@Mock
	private PolicyDeploymentPort policyDeploymentPort;

	@Captor
	private ArgumentCaptor<UUID> uuidArgumentCaptor;

	@Captor
	private ArgumentCaptor<Policy> policyArgumentCaptor;

	@Captor
	private ArgumentCaptor<Dataset> datasetArgumentCaptor;

	@Test
	void shouldDeployPolicyAndCreateNewDataset() {
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

		given(offerPersistencePort.findById(any())).willReturn(
				new Offer(
						uuid,
						metadata,
						List.of(dataRow)
				)
		);

		underTest.handle(new BuyOfferCommand(uuid));

		verify(datasetPersistencePort).save(datasetArgumentCaptor.capture());
		verify(policyDeploymentPort).deployPolicy(policyArgumentCaptor.capture());
		verify(offerPersistencePort).deleteById(uuidArgumentCaptor.capture());


		assertThat(policyArgumentCaptor.getValue().getId()).isEqualTo(9);
		assertThat(datasetArgumentCaptor.getValue().getMetadata().getId()).isEqualTo(42);
		assertThat(datasetArgumentCaptor.getValue().getOfferId()).isEqualTo(uuid);
		assertThat(uuidArgumentCaptor.getValue()).isEqualTo(uuid);
	}
}
