package de.thesis.provider.backend.integration;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.provider.backend.CompositionConfig;
import de.thesis.provider.backend.controllers.OfferController;
import de.thesis.provider.backend.csv.DataRow;
import de.thesis.provider.backend.dto.CreateOfferCommand;
import de.thesis.provider.backend.dto.Metadata;
import de.thesis.provider.backend.dto.Policy;
import de.thesis.provider.backend.httpclient.InsuranceClient;
import de.thesis.provider.backend.httpclient.SpringHttpClient;
import de.thesis.provider.backend.services.OfferService;
import lombok.extern.slf4j.Slf4j;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.http.MediaType;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.context.TestPropertySource;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.MvcResult;
import org.springframework.web.servlet.config.annotation.EnableWebMvc;

import java.util.List;
import java.util.UUID;

import static org.hamcrest.Matchers.hasSize;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@SpringBootTest
@AutoConfigureMockMvc
@ContextConfiguration(classes = {OfferController.class, OfferService.class, InsuranceClient.class, SpringHttpClient.class, CompositionConfig.class, ObjectMapper.class})
@TestPropertySource(properties = {"server.port = 8082", "consumer.url = http://localhost:8080"})
@EnableWebMvc
@Slf4j
public class AddOfferTest {

	@Autowired
	private ObjectMapper mapper;

	@Autowired
	private MockMvc mvc;

	@AfterEach
	private void cleanUpEach() throws Exception {
		MvcResult mvcResult = mvc.perform(get("/offers")
				)
				.andExpect(status().isOk())
				.andReturn();

		JsonNode node = mapper.readTree(mvcResult.getResponse().getContentAsString());
		node.fieldNames().forEachRemaining(offerId -> {
			if (!IntegrationTestUtils.isInitialOffer(UUID.fromString(offerId))) {
				try {
					mvc.perform(delete("/offers/" + offerId)
							)
							.andExpect(status().isOk());

					log.info("Offer {} deleted", offerId);
				} catch (Exception e) {
					throw new RuntimeException(e);
				}
			}
		});
	}

	@Test
	public void addOffer() throws Exception {
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

		mvc.perform(get("/offers")
				)
				.andExpect(status().isOk())
				// Initially, there are three offers, after adding one there are four
				.andExpect(jsonPath("$.*", hasSize(4)));
	}
}
