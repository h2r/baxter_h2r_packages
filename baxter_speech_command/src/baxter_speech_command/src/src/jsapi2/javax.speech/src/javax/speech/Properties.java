package javax.speech;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Hashtable;

class Properties extends Hashtable {
    public void load(InputStream input) throws IOException {
        InputStreamReader reader = new InputStreamReader(input, "ISO-8859-1");

        String line;

        while ((line = readLine(reader)) != null) {
            int pos = 0;
            char c = 0;
            // Leading whitespaces must be deleted first.
            while (pos < line.length() && isWhitespace(c = line.charAt(pos))) {
                ++pos;
            }

            // If empty line or begins with a comment character, skip this line.
            if ((line.length() - pos) == 0 || line.charAt(pos) == '#'
                    || line.charAt(pos) == '!') {
                continue;
            }

            // The characters up to the next Whitespace, ':', or '='
            // describe the key. But look for escape sequences.
            StringBuffer key = new StringBuffer();
            while (pos < line.length() && !isWhitespace(c = line.charAt(pos++))
                    && c != '=' && c != ':') {
                if (c == '\\') {
                    if (pos == line.length()) {
                        // The line continues on the next line. If there
                        // is no next line, just treat it as a key with an
                        // empty value.
                        line = readLine(reader);
                        if (line == null) {
                            line = "";
                        }
                        pos = 0;
                        while (pos < line.length()
                                && isWhitespace(c = line.charAt(pos))) {
                            pos++;
                        }
                    } else {
                        c = line.charAt(pos++);
                        switch (c) {
                        case 'n':
                            key.append('\n');
                            break;
                        case 't':
                            key.append('\t');
                            break;
                        case 'r':
                            key.append('\r');
                            break;
                        case 'u':
                            if (pos + 4 <= line.length()) {
                                char uni = (char) Integer.parseInt(line
                                        .substring(pos, pos + 4), 16);
                                key.append(uni);
                                pos += 4;
                            } 
                            break;
                        default:
                            key.append(c);
                            break;
                        }
                    }
                } else
                    key.append(c);
            }

            boolean isDelim = (c == ':' || c == '=');
            while (pos < line.length() && isWhitespace(c = line.charAt(pos))) {
                pos++;
            }

            if (!isDelim && (c == ':' || c == '=')) {
                pos++;
                while (pos < line.length()
                        && isWhitespace(c = line.charAt(pos)))
                    pos++;
            }

            StringBuffer element = new StringBuffer(line.length() - pos);
            while (pos < line.length()) {
                c = line.charAt(pos++);
                if (c == '\\') {
                    if (pos == line.length()) {
                        // The line continues on the next line.
                        line = readLine(reader);
                        if (line == null) {
                            break;
                        }

                        pos = 0;
                        while (pos < line.length()
                                && isWhitespace(c = line.charAt(pos))) {
                            pos++;
                        }
                        element.ensureCapacity(line.length() - pos
                                + element.length());
                    } else {
                        c = line.charAt(pos++);
                        switch (c) {
                        case 'n':
                            element.append('\n');
                            break;
                        case 't':
                            element.append('\t');
                            break;
                        case 'r':
                            element.append('\r');
                            break;
                        case 'u':
                            if (pos + 4 <= line.length()) {
                                char uni = (char) Integer.parseInt(line
                                        .substring(pos, pos + 4), 16);
                                element.append(uni);
                                pos += 4;
                            } 
                            break;
                        default:
                            element.append(c);
                            break;
                        }
                    }
                } else {
                    element.append(c);
                }
            }
            put(key.toString(), element.toString());
        }
    }

    public String getProperty(String key) {
        return (String) get(key);
    }

    static boolean isWhitespace(char ch) {
        switch (ch) {
        case ' ':
        case '\t':
            return true;
        default:
            return false;
        }
    }
    
    private String readLine(InputStreamReader reader) throws IOException {
        StringBuffer str = new StringBuffer();
        int read = 1;
        while (read > 0) {
            read = reader.read();
            if (read == '\n') {
                break;
            }
            if (read > 0) {
                str.append((char)read);
            }
        }
        
        if (str.length() == 0) {
            return null;
        }
        
        return str.toString();
    }
}
