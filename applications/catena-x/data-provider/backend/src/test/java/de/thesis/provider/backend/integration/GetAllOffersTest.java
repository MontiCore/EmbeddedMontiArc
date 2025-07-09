package de.thesis.provider.backend.integration;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.provider.backend.CompositionConfig;
import de.thesis.provider.backend.controllers.OfferController;
import de.thesis.provider.backend.httpclient.InsuranceClient;
import de.thesis.provider.backend.httpclient.SpringHttpClient;
import de.thesis.provider.backend.services.OfferService;
import lombok.extern.slf4j.Slf4j;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.context.TestPropertySource;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.MvcResult;
import org.springframework.web.servlet.config.annotation.EnableWebMvc;

import java.util.UUID;

import static org.assertj.core.api.AssertionsForClassTypes.assertThat;
import static org.hamcrest.Matchers.hasSize;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@SpringBootTest
@AutoConfigureMockMvc
@ContextConfiguration(classes = {OfferController.class, OfferService.class, InsuranceClient.class, SpringHttpClient.class, CompositionConfig.class, ObjectMapper.class})
@TestPropertySource(properties = {"server.port = 8082", "consumer.url = http://localhost:8080"})
@EnableWebMvc
@Slf4j
public class GetAllOffersTest {

	@Autowired
	private ObjectMapper mapper;

	@Autowired
	private MockMvc mvc;

	@Test
	public void getThreeOffers() throws Exception {
		MvcResult mvcResult = mvc.perform(get("/offers")
				)
				.andExpect(status().isOk())
				// Initially the consumer application contains three offers
				.andExpect(jsonPath("$.*", hasSize(3)))
				.andReturn();

		JsonNode node = mapper.readTree(mvcResult.getResponse().getContentAsString());
		node.fieldNames().forEachRemaining(offerId -> {
			assertThat(IntegrationTestUtils.isInitialOffer(UUID.fromString(offerId))).isTrue();
		});
	}
}
