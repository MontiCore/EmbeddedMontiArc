package de.thesis.provider.backend.unit.controllers;

import de.thesis.provider.backend.controllers.FileController;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.http.MediaType;
import org.springframework.mock.web.MockMultipartFile;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.context.TestPropertySource;
import org.springframework.test.web.servlet.MockMvc;

import static org.hamcrest.Matchers.hasSize;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.multipart;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@WebMvcTest(FileController.class)
@ContextConfiguration(classes = {FileController.class})
@TestPropertySource(properties = {"server.port = 8082"})
class FileControllerTest {

	@Autowired
	private MockMvc mvc;


	@Test
	void shouldReturnTwoDataRows() throws Exception {

		MockMultipartFile file
				= new MockMultipartFile(
				"file",
				"hello.txt",
				MediaType.TEXT_PLAIN_VALUE,
				("id;latitude;longitude;gpsTime;speed;heading;totalFuelUsed;dayId;timestamp;odometer\n" +
						"1611482441;50.672425;3.234842;2021-08-18T07:36:27Z;0;13;0;71919bfcd628df321666fc97aac1c23b501d195b096e38bdcc5b7ce98b9bdd34;2021-08-18T07:36:36.1062975Z;358850\n" +
						"1611538045;53.017757;10.739292;2021-08-18T11:31:51Z;67;23;0;deaa5ddce8433b459bd2e58cea13013907ce0c36d97894925271fc4093bafb6b;2021-08-18T11:31:53.0948647Z;114549\n" +
						"1611566774;50.68277;2.90924;2021-08-18T13:30:46Z;13;29;0;23b61e04f2b79148c317e349465c73e4b8dd270fb36c2f954322233af9cd0bdd;2021-08-18T13:30:48.4569464Z;-1\n" +
						"1611477727;50.6545;3.1302;2021-08-18T07:16:54Z;80;31;8636;b633f9f4b423c2b7583a71feb9e285b63849016caf39ab3a547a63be09f116c5;2021-08-18T07:16:57.0542591Z;880796").getBytes()
		);

		mvc.perform(multipart("/fileupload").file(file))
				.andExpect(status().isOk())
				.andExpect(jsonPath("$", hasSize(4)));
	}
}
