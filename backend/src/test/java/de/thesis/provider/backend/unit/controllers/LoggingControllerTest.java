package de.thesis.provider.backend.unit.controllers;

import de.thesis.provider.backend.controllers.LoggingController;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.WebMvcTest;
import org.springframework.boot.test.system.CapturedOutput;
import org.springframework.boot.test.system.OutputCaptureExtension;
import org.springframework.test.context.ContextConfiguration;
import org.springframework.test.context.TestPropertySource;
import org.springframework.test.web.servlet.MockMvc;

import static org.assertj.core.api.AssertionsForClassTypes.assertThat;
import static org.hamcrest.Matchers.hasSize;
import static org.junit.jupiter.api.Assertions.*;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.multipart;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@WebMvcTest(LoggingController.class)
@ContextConfiguration(classes = {LoggingController.class})
@TestPropertySource(properties = {"server.port = 8082"})
@ExtendWith(OutputCaptureExtension.class)
class LoggingControllerTest {

	@Autowired
	private MockMvc mvc;

	@Test
	void shouldLogDataAccess(CapturedOutput output) throws Exception {
		mvc.perform(post("/logging/e8f2ccf3-cc84-4d4f-9222-44760cd4c949"))
				.andExpect(status().isOk());

		assertThat(output.getAll().contains("Dataset e8f2ccf3-cc84-4d4f-9222-44760cd4c949 accessed")).isTrue();
	}
}
