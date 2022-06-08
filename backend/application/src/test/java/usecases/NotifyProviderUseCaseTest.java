package usecases;

import commands.NotifyProviderCommand;
import entity.Dataset;
import entity.Policy;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.ProviderCommunicationPort;

import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class NotifyProviderUseCaseTest {

	@Mock
	private ProviderCommunicationPort communicationPort;

	@InjectMocks
	private NotifyProviderUseCase underTest;

	@Test
	void shouldCallWithCorrectUrlAndId() {
		// when
		UUID uuid = UUID.fromString("63cc0e05-15a1-4751-8a2a-d201a47c11b1");
		ArgumentCaptor<String> urlArgumentCaptor =
				ArgumentCaptor.forClass(String.class);

		ArgumentCaptor<UUID> idArgumentCaptor =
				ArgumentCaptor.forClass(UUID.class);

		underTest.handle(new NotifyProviderCommand(
						"/logging",
						uuid
				)
		);

		// then
		verify(communicationPort).notifyProvider(urlArgumentCaptor.capture(), idArgumentCaptor.capture());
		assertThat(urlArgumentCaptor.getValue()).isEqualTo("/logging");
		assertThat(idArgumentCaptor.getValue()).isEqualTo(uuid);
	}
}
