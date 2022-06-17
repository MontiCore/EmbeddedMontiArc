package presentation.controllers;

import com.fasterxml.jackson.databind.ObjectMapper;
import commands.AddOfferCommand;
import commands.BuyOfferCommand;
import entity.DataRow;
import entity.Metadata;
import entity.Policy;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.http.MediaType;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.web.servlet.MockMvc;
import usecases.AddOfferUseCase;
import usecases.BuyOfferUseCase;
import usecases.GetOffersMetadataUseCase;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.hamcrest.collection.IsMapContaining.hasKey;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@WebMvcTest(OfferController.class)
@ContextConfiguration(classes = {OfferController.class, ObjectMapper.class})
class OfferControllerTest {

	@Autowired
	private MockMvc mvc;

	@Autowired
	private ObjectMapper mapper;

	@MockBean
	private AddOfferUseCase addOfferUseCase;

	@MockBean
	private BuyOfferUseCase buyOfferUseCase;

	@MockBean
	private GetOffersMetadataUseCase getAllOffersUseCase;

	@Test
	void shouldAddOffer() throws Exception {
		Policy policy = new Policy();
		policy.setMaxUsages(5);

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
				null,
				42,
				50,
				22000,
				50,
				null
		);

		AddOfferCommand command = new AddOfferCommand(
				metadata,
				List.of(dataRow)
		);

		ArgumentCaptor<AddOfferCommand> commandArgumentCaptor =
				ArgumentCaptor.forClass(AddOfferCommand.class);

		mvc.perform(post("/offers")
						.contentType(MediaType.APPLICATION_JSON)
						.content(mapper.writeValueAsString(command))
				)
				.andExpect(status().isOk());

		verify(addOfferUseCase).handle(commandArgumentCaptor.capture());

		assertThat(commandArgumentCaptor.getValue().getMetadata().getTitle()).isEqualTo("Aachen Dataset");
		assertThat(commandArgumentCaptor.getValue().getMetadata().getPolicy().getMaxUsages()).isEqualTo(5);
	}

	@Test
	void shouldCallUseCaseWithCorrectId() throws Exception {
		UUID uuid = UUID.randomUUID();

		ArgumentCaptor<BuyOfferCommand> commandArgumentCaptor =
				ArgumentCaptor.forClass(BuyOfferCommand.class);

		mvc.perform(post("/offers/" + uuid)
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk());

		verify(buyOfferUseCase).handle(commandArgumentCaptor.capture());
		assertThat(commandArgumentCaptor.getValue().getOfferId()).isEqualTo(uuid);
	}

	@Test
	void shouldReturnMapWithTwoOffers() throws Exception {
		// given
		Map<UUID, Metadata> map = new HashMap<>();
		Policy firstPolicy = new Policy();
		firstPolicy.setMaxUsages(2);

		Policy secondPolicy = new Policy();
		secondPolicy.setMaxUsages(10);

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

		UUID firstId = UUID.randomUUID();
		UUID secondId = UUID.randomUUID();
		map.put(firstId, firstMetadata);
		map.put(secondId, secondMetadata);
		given(getAllOffersUseCase.handle(any())).willReturn(map);

		mvc.perform(get("/offers")
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk())
				.andExpect(jsonPath("$", hasKey(firstId.toString())))
				.andExpect(jsonPath("$", hasKey(secondId.toString())));
	}
}
