package datasovereignty;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import ports.HttpClientPort;

import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.verify;

@ExtendWith(MockitoExtension.class)
class ProviderCommunicationPortAdapterTest {

	@InjectMocks
	private ProviderCommunicationPortAdapter underTest;

	@Mock
	private HttpClientPort clientPort;

	@Captor
	private ArgumentCaptor<String> stringArgumentCaptor;

	@Test
	void shouldUseCorrectLoggingUrl() {
		underTest.notifyProvider("/logging", UUID.fromString("d2f22172-60e8-4bdc-84f2-0f00521d625d"));

		verify(clientPort).post(stringArgumentCaptor.capture(), any(), any());

		assertThat(stringArgumentCaptor.getValue()).isEqualTo("/logging/d2f22172-60e8-4bdc-84f2-0f00521d625d");
	}
}
