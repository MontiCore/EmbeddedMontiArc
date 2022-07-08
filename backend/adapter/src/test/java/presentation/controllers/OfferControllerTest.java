package presentation.controllers;

import com.fasterxml.jackson.databind.ObjectMapper;
import commands.AddOfferCommand;
import commands.BuyOfferCommand;
import commands.DeleteOfferCommand;
import entity.DataRow;
import entity.Metadata;
import entity.Policy;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.http.MediaType;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.web.servlet.MockMvc;
import usecases.AddOfferUseCase;
import usecases.BuyOfferUseCase;
import usecases.DeleteOfferUseCase;
import usecases.GetAllOffersMetadataUseCase;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.hamcrest.collection.IsMapContaining.hasKey;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
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
	private GetAllOffersMetadataUseCase getAllOffersUseCase;

	@MockBean
	private DeleteOfferUseCase deleteOfferUseCase;

	@Captor
	private ArgumentCaptor<AddOfferCommand> addOfferCommandArgumentCaptor;

	@Captor
	private ArgumentCaptor<BuyOfferCommand> buyOfferCommandArgumentCaptor;

	@Captor
	private ArgumentCaptor<DeleteOfferCommand> deleteOfferCommandArgumentCaptor;

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

		mvc.perform(post("/offers")
						.contentType(MediaType.APPLICATION_JSON)
						.content(mapper.writeValueAsString(new AddOfferCommand(metadata, List.of(dataRow))))
				)
				.andExpect(status().isOk());

		verify(addOfferUseCase).handle(addOfferCommandArgumentCaptor.capture());

		assertThat(addOfferCommandArgumentCaptor.getValue().getMetadata().getTitle()).isEqualTo("Aachen Dataset");
		assertThat(addOfferCommandArgumentCaptor.getValue().getMetadata().getPolicy().getMaxUsages()).isEqualTo(5);
	}

	@Test
	void shouldCallUseCaseWithCorrectId() throws Exception {
		UUID uuid = UUID.randomUUID();

		mvc.perform(post("/offers/" + uuid)
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk());

		verify(buyOfferUseCase).handle(buyOfferCommandArgumentCaptor.capture());
		assertThat(buyOfferCommandArgumentCaptor.getValue().getOfferId()).isEqualTo(uuid);
	}

	@Test
	void shouldReturnMapWithTwoOffers() throws Exception {
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

	@Test
	void shouldDeleteOffer() throws Exception {
		UUID uuid = UUID.randomUUID();

		mvc.perform(delete("/offers/4875c6f3-60be-4fbc-ba8e-4b4e2637846a")
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk());

		verify(deleteOfferUseCase).handle(deleteOfferCommandArgumentCaptor.capture());

		assertThat(deleteOfferCommandArgumentCaptor.getValue().getOfferId())
				.isEqualTo(UUID.fromString("4875c6f3-60be-4fbc-ba8e-4b4e2637846a"));
	}
}
