package Files;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

public class JSON {
	public static class JSONObject extends HashMap<String, Object> {
		public String stringfy() {
			String str = "";
			for(String key : keySet()) {
				Object value = get(key);
				if(value.getClass().equals(String.class)) {
					value = (String) ("\"" + value + "\""); 
				}else if(value.getClass().equals(JSONObject.class)) {
					value = (String) ((JSONObject) value).stringfy();
				}else if(value.getClass().equals(JSONArray.class)) {
					value = (String) ((JSONArray) value).stringfy();
				}
				str += "\"" + key + "\"" + " : " + value + ",\n";
			}
			str = "{\n" + str.substring(0, str.length() - 2) + "\n}";
			return str;
		}
		
		public static JSONObject read(String str) {
			JSONObject obj = new JSONObject();
			String[] rawLines = str.split("\n");
			for(int i = 1; i < rawLines.length - 1; i++) {
				String line = rawLines[i];
				String key = line.substring(1, line.indexOf(":") - 2); 
				Object value = null;
				String valueStr = line.substring(line.indexOf(":") + 2, line.length());
				if(valueStr.charAt(valueStr.length() - 1) == ',') {
					valueStr = valueStr.substring(0, valueStr.length() - 1);
				}
				if(valueStr.contains("[")) {
					if(valueStr.contains("]")) {
						// Does not have an object inside
						value = (JSONArray) JSONArray.read(valueStr);
					}else {
						// Has a object inside
						String arrData = valueStr + "\n";
						do {
							i++;
							arrData += rawLines[i] + "\n";
						}
						while(!rawLines[i].contains("]"));
						value = (JSONArray) JSONArray.read(arrData);
					}
				}else if(valueStr.contains("{")) {
					String objData = valueStr + "\n";
					do {
						i++;
						objData += rawLines[i] + "\n";
					}
					while(!rawLines[i].contains("}"));
					value = (JSONObject) read(objData);
				}else if(valueStr.contains("\"")) {
					value = (String) valueStr.substring(1, valueStr.length() - 1);
				}else if(valueStr.contains("true") || valueStr.contains("false")) {
					value = (Boolean) Boolean.parseBoolean(valueStr);
				}else if(valueStr.contains(".")) {
					value = (Double) Double.parseDouble(valueStr);
				}else {
					value = Integer.parseInt(valueStr);
				}
				obj.put(key, value);
			}
			return obj;
		}
	}
	
	public static class JSONArray extends ArrayList<Object> {
		public String stringfy() {
			String arr = ""; 
			for(int i = 0; i < size(); i++) {
				 Object element = get(i);
				 if(element.getClass().equals(String.class)) {
					 element = (String) "\"" + element + "\"";
				 }else if(element.getClass().equals(JSONObject.class)) {
					 element = (String) ((JSONObject) element).stringfy();
				 }else if(element.getClass().equals(JSONArray.class)) {
					 element = (String) ((JSONArray) element).stringfy();
				 }
				 arr += (i == size() - 1) ? element : element + ", " ;
			}
			arr = "[" + arr + "]";
			return arr;
		}
		
		public static JSONArray read(String str) {
			JSONArray arr = new JSONArray();
			String[] elementsStr = str.split(", "); 
			elementsStr[0] = elementsStr[0].substring(1, elementsStr[0].length());
			String last = elementsStr[elementsStr.length - 1];
			last = last.substring(0, last.length() - 1);
			elementsStr[elementsStr.length - 1] = last;
			for(String elementStr : elementsStr) {
				Object element = null;
				if(elementStr.contains("[")) {
					element = (JSONArray) read(elementStr);
				}else if(elementStr.contains("{")) {
					element = (JSONObject) JSONObject.read(elementStr);
				}else if(elementStr.contains("\"")) {
					element = (String) elementStr.substring(1, elementStr.length() - 1);
				}else if(elementStr.contains("true") || elementStr.contains("false")) {
					element = (Boolean) Boolean.parseBoolean(elementStr);
				}else if(elementStr.contains(".")) {
					element = (Double) Double.parseDouble(elementStr);
				}else {
					element = (Integer) Integer.parseInt(elementStr);
				}
				arr.add(element);
			}
			return arr;
		}
	}
	
	public static void writeJSONFile(JSONObject data, String path) throws IOException {
		if(!path.substring(path.length() - 5, path.length()).equals(".json")) {
			path += ".json";
		}
		File file = new File(path);
		FileWriter fWriter = new FileWriter(file);
		BufferedWriter bWriter = new BufferedWriter(fWriter);
		
		bWriter.write(data.stringfy());
		bWriter.flush();
		bWriter.close();
	}
	
	public static JSONObject readJSONFile(String path) throws IOException {
		File file = new File(path);
		FileReader fReader = new FileReader(file);
		BufferedReader bReader = new BufferedReader(fReader);
		
		String data = ""; 
		String next = bReader.readLine();
		while(next != null) {
			data += next + "\n";
			next = bReader.readLine();
		}
		return JSONObject.read(data);
	}
}