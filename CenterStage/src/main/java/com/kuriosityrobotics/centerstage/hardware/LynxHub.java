package com.kuriosityrobotics.centerstage.hardware;

public enum LynxHub {
	CONTROL_HUB("Control Hub"),
	EXPANSION_HUB("Expansion Hub");

	private final String hardwareName;

	LynxHub(String hardwareName) {
		this.hardwareName = hardwareName;
	}

	public String hardwareName() {
		return hardwareName;
	}
}
