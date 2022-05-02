package de.thesis.provider.backend.policy;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalTime;

@Getter
@Setter
public class BusinessHours {
	private LocalTime start;
	private LocalTime end;
}
