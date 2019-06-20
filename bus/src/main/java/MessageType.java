public enum MessageType {
	SEND("SEND"), 
	RECEIVE("RECEIVE");
	/**
	 * this variable contains the name of the entry
	 */
	private final String name;

	/**
     * constructor for a bus entry
     *
     * @param name the name of the entry
     */
    private MessageType(String name) {
        this.name = name;
    }

	/**
	 * Getter of the name attribute
	 *
	 * @return the name of the entry
	 */
	public String toString() {
		return this.name;
	}
}
