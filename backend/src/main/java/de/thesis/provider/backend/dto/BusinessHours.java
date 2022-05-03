package de.thesis.provider.backend.dto;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalTime;

@Getter
@Setter
public class BusinessHours {
	private LocalTime start;
	private LocalTime end;
}
