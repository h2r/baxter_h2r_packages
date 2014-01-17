package org.jvoicexml.jsapi2.recognition;

import javax.speech.recognition.Result;
import javax.speech.recognition.ResultToken;
import javax.speech.recognition.RecognizerProperties;

/**
 * Basic implementation of a {@link ResultToken}.
 *
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version 1.0
 */
public class BaseResultToken implements ResultToken {

    private int confidenceLevel;
    private long startTime;
    private long endTime;
    private final Result result;
    private final String token;

    /**
     * Constructs a new object
     * @param res the result
     * @param tok the token
     */
    public BaseResultToken(final Result res, final String tok) {
        result = res;
        confidenceLevel = RecognizerProperties.NORM_CONFIDENCE;
        startTime = -1;
        endTime = -1;
        token = tok;
    }

    /**
     * Retrieves the confidence level if the result was accepted. 
     *
     * @return the confidence level,
     *          <code>RecognizerProperties.UNKNOWN_CONFIDENCE</code> if the
     *          result was not accepted.
     */
    public int getConfidenceLevel() {
        if (result.getResultState() == Result.ACCEPTED) {
            return confidenceLevel;
        }

        return RecognizerProperties.UNKNOWN_CONFIDENCE;
    }

    /**
     * Sets the confidence level.
     * @param the confidence level
     */
    public void setConfidenceLevel(final int level) {
        confidenceLevel = level;
    }

    /**
     * Sets the start time.
     * @param time the start time
     */
    public void setStartTime(final long time) {
        startTime = time;
    }

    /**
     * Retrieves the start time.
     *
     * @return the start time
     */
    public long getStartTime() {
        return startTime;
    }

    /**
     * Sets the end time.
     * @param time the end time
     */
    public void setEndTime(final long time) {
        endTime = time;
    }

    /**
     * Retrieves the end time.
     *
     * @return the end time.
     */
    public long getEndTime() {
        return endTime;
    }

    /**
     * Retrieves the result.
     *
     * @return the result.
     */
    public Result getResult() {
        return result;
    }

    /**
     * Retrieves the result token.
     *
     * @return the token
     */
    public String getText() {
        return token;
    }
}
