package usecases;

import commands.RemoteLoggingCommand;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.ProviderCommunicationPort;

import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class RemoteLoggingUseCaseTest {

	@InjectMocks
	private RemoteLoggingUseCase underTest;

	@Mock
	private ProviderCommunicationPort communicationPort;

	@Test
	void shouldCallCommunicationPortWithCorrectParams() {

		ArgumentCaptor<String> stringArgumentCaptor =
				ArgumentCaptor.forClass(String.class);
		ArgumentCaptor<UUID> uuidArgumentCaptor =
				ArgumentCaptor.forClass(UUID.class);
		underTest.handle(new RemoteLoggingCommand("/log", UUID.fromString("8a484636-e817-4fa9-9e7e-7300c799547b")));

		verify(communicationPort).notifyProvider(stringArgumentCaptor.capture(), uuidArgumentCaptor.capture());

		assertThat(stringArgumentCaptor.getValue()).isEqualTo("/log");
		assertThat(uuidArgumentCaptor.getValue()).isEqualTo(UUID.fromString("8a484636-e817-4fa9-9e7e-7300c799547b"));
	}
}
