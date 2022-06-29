package presentation.controllers;

import commands.DeleteDatasetCommand;
import dto.DatasetViewDto;
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
import queries.GetDatasetQuery;
import usecases.DeleteDatasetUseCase;
import usecases.GetAllDatasetMetadataUseCase;
import usecases.GetDatasetViewUseCase;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;
import static org.hamcrest.collection.IsMapContaining.hasKey;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.BDDMockito.given;
import static org.mockito.Mockito.verify;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.delete;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@WebMvcTest(DatasetController.class)
@ContextConfiguration(classes = {DatasetController.class})
class DatasetControllerTest {

	@Autowired
	private MockMvc mvc;

	@MockBean
	private DeleteDatasetUseCase deleteDatasetUseCase;

	@MockBean
	private GetDatasetViewUseCase getDatasetViewUseCase;

	@MockBean
	private GetAllDatasetMetadataUseCase getAllDatasetMetadataUseCase;

	@Captor
	private ArgumentCaptor<DeleteDatasetCommand> commandArgumentCaptor;

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

		DatasetViewDto datasetViewDto = new DatasetViewDto(
				uuid,
				metadata,
				List.of(dataRow),
				null,
				1,
				0,
				1,
				50, 3550.99
		);

		given(getDatasetViewUseCase.handle(new GetDatasetQuery(uuid))).willReturn(datasetViewDto);

		mvc.perform(get("/datasets/" + uuid)
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk())
				.andExpect(jsonPath("$.id").value("d8e71f74-1c7e-4f69-a9db-786d65f7e17a"))
				.andExpect(jsonPath("$.metadata.title").value("Aachen Dataset"));
	}

	@Test
	void shouldDeleteCorrectDataset() throws Exception {
		mvc.perform(delete("/datasets/73816ebc-bcf5-41dd-a070-d69e12729e5f")
						.contentType(MediaType.APPLICATION_JSON))
				.andExpect(status().isOk());

		verify(deleteDatasetUseCase).handle(commandArgumentCaptor.capture());
		assertThat(commandArgumentCaptor.getValue().getDatasetId()).isEqualTo(UUID.fromString("73816ebc-bcf5-41dd-a070-d69e12729e5f"));
	}
}
