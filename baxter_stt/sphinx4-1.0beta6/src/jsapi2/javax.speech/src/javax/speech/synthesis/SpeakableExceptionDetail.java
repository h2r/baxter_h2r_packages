package javax.speech.synthesis;

//Comp 2.0.6

public class SpeakableExceptionDetail {
    public static int UNKNOWN_TYPE = -1;
    public static int UNKNOWN_VALUE = -1;
    public static int UNSUPPORTED_ALPHABET = 0x1;
    public static int UNSUPPORTED_AUDIO = 0x2;
    public static int UNSUPPORTED_INTERPRETATION = 0x3;
    public static int UNSUPPORTED_LANGUAGE = 0x4;
    public static int UNSUPPORTED_LEXEME = 0x5;
    public static int UNSUPPORTED_LEXICON = 0x6;
    public static int UNSUPPORTED_PHONEME = 0x7;
    public static int UNSUPPORTED_VOICE = 0x8;
    public static int SYNTAX_ERROR = 0x9;

    private final int type;
    private final String textInfo;
    private final int lineNumber;
    private final int charNumber;
    private final String message;

    public SpeakableExceptionDetail(int type, String textInfo, int lineNumber,
            int charNumber, String message) throws IllegalArgumentException {
        if ((lineNumber < 0) && (lineNumber != UNKNOWN_VALUE)) {
            throw new IllegalArgumentException(
                    "Line number must be a positive numer or UNKNOWN_VALUE!");
        }
        if ((charNumber < 0) && (charNumber != UNKNOWN_VALUE)) {
            throw new IllegalArgumentException(
                    "Char number must be a positive numer or UNKNOWN_VALUE!");
        }
        this.type = type;
        this.textInfo = textInfo;
        this.lineNumber = lineNumber;
        this.charNumber = charNumber;
        this.message = message;
    }

    public int getLineNumber() {
        return lineNumber;
    }

    public int getCharNumber() {
        if (getLineNumber() == UNKNOWN_VALUE) {
            return UNKNOWN_VALUE;
        }
        return charNumber;
    }

    public String getMessage() {
        return message;
    }

    public String getTextInfo() {
        return textInfo;
    }

    public int getType() {
        return type;
    }
}
