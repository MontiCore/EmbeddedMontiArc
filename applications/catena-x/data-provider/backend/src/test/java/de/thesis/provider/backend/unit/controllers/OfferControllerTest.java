package de.thesis.provider.backend.unit.controllers;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.provider.backend.controllers.OfferController;
import de.thesis.provider.backend.csv.DataRow;
import de.thesis.provider.backend.dto.CreateOfferCommand;
import de.thesis.provider.backend.dto.Metadata;
import de.thesis.provider.backend.dto.Policy;
import de.thesis.provider.backend.services.OfferService;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Captor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.http.MediaType;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.context.TestPropertySource;
import org.springframework.test.web.servlet.MockMvc;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.hamcrest.Matchers.is;
import static org.hamcrest.collection.IsMapContaining.hasKey;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@WebMvcTest(OfferController.class)
@ContextConfiguration(classes = {OfferController.class, OfferService.class, ObjectMapper.class})
@TestPropertySource(properties = {"server.port = 8082"})
class OfferControllerTest {

	@Autowired
	private MockMvc mvc;

	@Autowired
	private ObjectMapper mapper;

	@MockBean
	private OfferService offerService;

	@Captor
	private ArgumentCaptor<CreateOfferCommand> commandArgumentCaptor;

	@Captor
	private ArgumentCaptor<UUID> uuidArgumentCaptor;

	@Test
	void shouldCallCreateOfferWithCorrectCommand() throws Exception {
		Policy policy = new Policy();
		policy.setMaxUsages(5);

		Metadata metadata = new Metadata();
		metadata.setTitle("Aachen Dataset");
		metadata.setProvider("Carrier GmbH");
		metadata.setDescription("A simple test data set...");
		metadata.setPrice(10);
		metadata.setLoggingUrl("/logging");
		metadata.setPolicy(policy);

		DataRow dataRow = new DataRow();
		dataRow.setId(1);
		dataRow.setDayID("1234");
		dataRow.setLongitude(51);
		dataRow.setLatitude(10);
		dataRow.setHeading(90);
		dataRow.setSpeed(30);
		dataRow.setOdometer(20000);
		dataRow.setTotalFuelUsed(20);

		mvc.perform(post("/offers")
						.contentType(MediaType.APPLICATION_JSON)
						.content(mapper.writeValueAsString(new CreateOfferCommand(metadata, List.of(dataRow))))
				)
				.andExpect(status().isOk());

		verify(offerService).addOffer(commandArgumentCaptor.capture());

		assertThat(commandArgumentCaptor.getValue().getMetadata().getTitle()).isEqualTo("Aachen Dataset");
		assertThat(commandArgumentCaptor.getValue().getData().size()).isEqualTo(1);
	}


	@Test
	void shouldReturnTwoOffers() throws Exception {
		Policy firstPolicy = new Policy();
		firstPolicy.setMaxUsages(5);

		Metadata firstMetadata = new Metadata();
		firstMetadata.setTitle("Friedrichshafen Dataset");
		firstMetadata.setProvider("Carrier GmbH");
		firstMetadata.setDescription("A simple test data set...");
		firstMetadata.setPrice(10);
		firstMetadata.setLoggingUrl("/logging");
		firstMetadata.setPolicy(firstPolicy);

		Policy secondPolicy = new Policy();
		secondPolicy.setMaxUsages(2);

		Metadata secondMetadata = new Metadata();
		secondMetadata.setTitle("Aachen Dataset");
		secondMetadata.setProvider("Carrier GmbH");
		secondMetadata.setDescription("Another test data set...");
		secondMetadata.setPrice(9.99);
		secondMetadata.setLoggingUrl("/logging");
		secondMetadata.setPolicy(secondPolicy);

		Map<UUID, Metadata> map = new HashMap<>();
		map.put(UUID.fromString("01b36eff-1267-4097-b818-d1f8c61a2f24"), firstMetadata);
		map.put(UUID.fromString("b3f2255f-eeac-41b3-b8ec-1e6f52766059"), secondMetadata);
		given(offerService.getOffers()).willReturn(map);

		mvc.perform(get("/offers")
						.contentType(MediaType.APPLICATION_JSON)
				)
				.andExpect(status().isOk())
				.andExpect(jsonPath("$", hasKey("01b36eff-1267-4097-b818-d1f8c61a2f24")))
				.andExpect(jsonPath("$", hasKey("b3f2255f-eeac-41b3-b8ec-1e6f52766059")))
				.andExpect(jsonPath("$['01b36eff-1267-4097-b818-d1f8c61a2f24'].title", is("Friedrichshafen Dataset")))
				.andExpect(jsonPath("$['b3f2255f-eeac-41b3-b8ec-1e6f52766059'].title", is("Aachen Dataset")));
	}

	@Test
	void shouldCallDeleteWithCorrectId() throws Exception {
		mvc.perform(delete("/offers/b1d18cfa-035f-4ba3-b14d-c2b295f396dc")
						.contentType(MediaType.APPLICATION_JSON)
				)
				.andExpect(status().isOk()
				);

		verify(offerService).deleteOffer(uuidArgumentCaptor.capture());

		assertThat(uuidArgumentCaptor.getValue().toString()).isEqualTo("b1d18cfa-035f-4ba3-b14d-c2b295f396dc");
	}
}
