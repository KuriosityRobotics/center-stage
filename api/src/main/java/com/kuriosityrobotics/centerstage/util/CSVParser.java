package com.kuriosityrobotics.centerstage.util;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;
import java.util.function.Consumer;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class CSVParser {
	private final Map<String, Integer> columnIndices;
	private final String[][] rows;

    private CSVParser(Map<String, Integer> columnIndices, String[][] rows) {
        this.columnIndices = columnIndices;
        this.rows = rows;
    }

	public void forEach(Consumer<CSVRecord> consumer) {
		for (int i = 0; i < rows.length; i++) {
			consumer.accept(new CSVRecord(i));
		}
	}

	public Stream<CSVRecord> stream() {
		return IntStream.iterate(0, i -> i < rows.length, i -> i + 1).mapToObj(CSVRecord::new);
	}

	public final class CSVRecord {
		private final int i;

        private CSVRecord(int i) {
            this.i = i;
        }

		public String get(String columnName) {
			if (!columnIndices.containsKey(columnName))
				throw new IllegalArgumentException("Column is not present.  Valid columns: " + columnIndices.keySet());

			return rows[i][columnIndices.get(columnName)];
		}
    }

    public static CSVParser parse(InputStream reader) {
		var scanner = new Scanner(reader);

		var columnNames = scanner.nextLine().split(",\\s*");
		var columnIndices = new HashMap<String, Integer>();

		for (int i = 0; i < columnNames.length; i++) {
			columnIndices.put(columnNames[i], i);
		}

		var rows = new ArrayList<String[]>();
		while (scanner.hasNextLine()) {
			rows.add(scanner.nextLine().split(",\\s*"));
		}

		return new CSVParser(columnIndices, rows.toArray(new String[0][0]));
	}
}
