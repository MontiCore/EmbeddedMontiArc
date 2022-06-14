package presentation.controllers;

import com.fasterxml.jackson.databind.ObjectMapper;
import commands.AddOfferCommand;
import commands.BuyOfferCommand;
import entity.*;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.http.MediaType;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.web.servlet.MockMvc;
import queries.GetAllDatasetMedataQuery;
import queries.GetDatasetQuery;
import usecases.*;

import java.time.LocalDateTime;
import java.time.LocalTime;
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
import static org.springframework.test.web.servlet.result.StatusResultMatchersExtensionsKt.isEqualTo;

@WebMvcTest(DatasetController.class)
@ContextConfiguration(classes = {DatasetController.class})
class DatasetControllerTest {

	@Autowired
	private MockMvc mvc;

	@MockBean
	private GetDatasetUseCase getDatasetUseCase;

	@MockBean
	private GetAllDatasetMetadataUseCase getAllDatasetMetadataUseCase;

	@Test
	void shouldReturnTwoDatasetMetadata() throws Exception {
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
		given(getAllDatasetMetadataUseCase.handle(any())).willReturn(map);

		mvc.perform(get("/datasets")
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk())
				.andExpect(jsonPath("$", hasKey(firstId.toString())))
				.andExpect(jsonPath("$", hasKey(secondId.toString())));
	}

	@Test
	void shouldReturnCorrectDataset() throws Exception {
		UUID uuid = UUID.fromString("d8e71f74-1c7e-4f69-a9db-786d65f7e17a");
		Policy policy = new Policy();
		policy.setMaxUsages(3);

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
				null,
				42,
				50,
				22000,
				50,
				null
		);

		Dataset dataset = new Dataset(
				uuid,
				new Offer(UUID.randomUUID(),
						metadata,
						List.of(dataRow)
				),
				metadata,
				List.of(dataRow),
				null
		);

		given(getDatasetUseCase.handle(new GetDatasetQuery(uuid))).willReturn(dataset);

		mvc.perform(get("/datasets/" + uuid)
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk())
				.andExpect(jsonPath("$.id").value("d8e71f74-1c7e-4f69-a9db-786d65f7e17a"))
				.andExpect(jsonPath("$.metadata.title").value("Aachen Dataset"));
	}
}
