package de.thesis.provider.backend.dto;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalDate;
import java.time.LocalTime;

@Getter
@Setter
public class Policy {
	private LocalTime startTime;
	private LocalTime endTime;
	private LocalDate expiresOn;
	private Integer maxUsages;
	private boolean localLogging;
	private boolean remoteLogging;
}
