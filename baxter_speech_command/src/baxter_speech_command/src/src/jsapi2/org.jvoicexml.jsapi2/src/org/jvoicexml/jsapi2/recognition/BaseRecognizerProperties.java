/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2/src/org/jvoicexml/jsapi2/BaseSpeechEventExecutor.java $
 * Version: $LastChangedRevision: 274 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007-2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.recognition;

import javax.speech.recognition.Recognizer;
import javax.speech.recognition.RecognizerProperties;

import org.jvoicexml.jsapi2.BaseEngineProperties;

/**
 * Base implementation of {@link RecognizerProperties}.
 *
 * <p>
 * Actual JSAPI2 implementations may want to override this class to
 * apply the settings to the recognizer.
 * </p>

 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version $Revision: $
 */
public class BaseRecognizerProperties
    extends BaseEngineProperties implements RecognizerProperties {
    /** Name of the training provided property in events. */
    public static final String TRAINING_PROVIDED = "trainingProvided";

    /** Name of the result audio provided property in events. */
    public static final String RESULT_AUDIO_PROVIDED = "resultAudioProvided";

    /** Name of the speed vs. accuracy property in events. */
    public static final String SPEED_VS_ACCURACY = "speedVsAccuracy";

    /** Name of the sensitivity property in events. */
    public static final String SENSITIVITY = "sensitivity";

    /** Name of the num result alternatives property in events. */
    public static final String NUM_RESULT_ALTERNATIVES =
        "numResultAlternatives";

    /** Name of the incomplete timeout property in events. */
    public static final String INCOMPLETE_TIMEOUT = "incompleteTimeout";

    /** Name of the endpoint style property in events. */
    public static final String ENDPOINT_STYLE = "endpointStyle";

    /** Name of the confidence threshold property in events. */
    public static final String CONFIDENCE_THRESHOLD = "confidenceThreshold";

    /** Name of the complete timeout property in events. */
    public static final String COMPLETE_TIMEOUT = "completeTimeout";

    /** Name of the adaptation property in events. */
    public static final String ADAPTATION = "adaptation";

    /**
     * Value used to control adaptation behavior. This value determines when a
     * Recognizer may adapt to the audio signal.
     */
    private int adaptation;

    /**
     * This timeout value, in milliseconds, determines the length of silence
     * required following user speech before the Recognizer  finalizes a Result.
     */
    private int completeTimeout;

    /**
     * The confidenceThreshold value can vary between MIN_CONFIDENCE and
     * MAX_CONFIDENCE. A value of NORM_CONFIDENCE is the default for the
     * Recognizer.
     */
    private int confidenceThreshold;

    /**
     * This value determines how a Recognizer knows when an utterance begins and
     * ends.
     */
    private int endpointStyle;

    /**
     * The incompleteTimeout value, in milliseconds, determines the required
     * length of silence following user speech after which a Recognizer
     * finalizes a Result.
     */
    private int incompleteTimeout;

    /**
     * This property indicates the preferred maximum number of N-best
     * alternatives in FinalResult objects.
     */
    private int numResultAlternatives;

    /**
     * The sensitivity can vary between MIN_SENSITIVITY and MAX_SENSITIVITY. A
     * value of NORM_SENSITIVITY is the default for the Recognizer. A value
     * of MAX_SENSITIVITY makes the Recognizer more sensitive to quiet input,
     * but also more sensitive to noise. A value of MIN_SENSITIVITY may require
     * the user to speak louder, but makes the Recognizer less sensitive to
     * background noise.
     */
    private int sensitivity;

    /**
     * A value of NORM_ACCURACY is the default compromise between speed and
     * accuracy for the Recognizer. A value of MIN_ACCURACY minimizes response
     * time. A value of MAX_ACCURACY maximizes recognition accuracy.
     */
    private int speedVsAccuracy;

    /**
     * If set to true, the Recognizer is requested to provide audio with
     * FinalResult objects.
     */
    private boolean resultAudioProvided;

    /**
     * If true, request a Recognizer to provide training information for
     * FinalResult objects.
     */
    private boolean trainingProvided;

    /**
     * Constructs a new object.
     * @param recognizer reference to the recognizer
     */
    public BaseRecognizerProperties(final Recognizer recognizer) {
        super(recognizer);
        adaptation = ADAPT_PAUSED | ADAPT_RESUMED;
        completeTimeout = 500;
        confidenceThreshold = NORM_CONFIDENCE;
        endpointStyle = ENDPOINT_SPEECH_DETECTION;
        incompleteTimeout = 1000;
        numResultAlternatives = 1;
        sensitivity = NORM_ACCURACY;
        speedVsAccuracy = NORM_ACCURACY;
        resultAudioProvided = false;
        trainingProvided = false;
    }

    /**
     * {@inheritDoc}
     */
    public int getAdaptation() {
        return adaptation;
    }

    /**
     * {@inheritDoc}
     */
    public int getCompleteTimeout() {
        return completeTimeout;
    }

    /**
     * {@inheritDoc}
     */
    public int getConfidenceThreshold() {
        return confidenceThreshold;
    }

    /**
     * {@inheritDoc}
     */
    public int getEndpointStyle() {
        return endpointStyle;
    }

    /**
     * {@inheritDoc}
     */
    public int getIncompleteTimeout() {
        return incompleteTimeout;
    }

    /**
     * {@inheritDoc}
     */
    public int getNumResultAlternatives() {
        return numResultAlternatives;
    }

    /**
     * {@inheritDoc}
     */
    public int getSensitivity() {
        return sensitivity;
    }

    /**
     * {@inheritDoc}
     */
    public int getSpeedVsAccuracy() {
        return speedVsAccuracy;
    }

    /**
     * {@inheritDoc}
     */
    public boolean isResultAudioProvided() {
        return resultAudioProvided;
    }

    /**
     * {@inheritDoc}
     */
    public boolean isTrainingProvided() {
        return trainingProvided;
    }

    /**
     * {@inheritDoc}
     */
    public void reset() {
        setAdaptation(ADAPT_PAUSED | ADAPT_RESUMED);
        setCompleteTimeout(500);
        setConfidenceThreshold(NORM_CONFIDENCE);
        setEndpointStyle(ENDPOINT_SPEECH_DETECTION);
        setIncompleteTimeout(1000);
        setNumResultAlternatives(1);
        setSensitivity(NORM_ACCURACY);
        setSpeedVsAccuracy(NORM_ACCURACY);
        setResultAudioProvided(false);
        setTrainingProvided(false);

        super.reset();
    }

    /**
     * {@inheritDoc}
     */
    public void setAdaptation(final int adapt) {
        postPropertyChangeEvent(ADAPTATION, new Integer(this.adaptation),
                                new Integer(adapt));
    }

    /**
     * {@inheritDoc}
     */
    public void setCompleteTimeout(final int value) {
        if (value < 0) {
            throw new IllegalArgumentException(
                    "Invalid completeTimeout: " + value);
        }

        postPropertyChangeEvent(COMPLETE_TIMEOUT,
                                new Integer(completeTimeout),
                                new Integer(value));
    }

    /**
     * {@inheritDoc}
     */
    public void setConfidenceThreshold(final int threshold) {
        if ((confidenceThreshold > MAX_CONFIDENCE)
            || (confidenceThreshold < MIN_CONFIDENCE)) {
            throw new IllegalArgumentException("Invalid confidenceThreshold: "
                                               + threshold);
        }
        postPropertyChangeEvent(CONFIDENCE_THRESHOLD,
                                new Integer(confidenceThreshold),
                                new Integer(threshold));
    }

    /**
     * {@inheritDoc}
     */
    public void setEndpointStyle(final int style) {
        if ((style != ENDPOINT_SPEECH_DETECTION)
            && (style != ENDPOINT_PUSH_TO_START)
            && (style != ENDPOINT_PUSH_TO_TALK)) {
            throw new IllegalArgumentException("Invalid endpointStyle: "
                                               + style);
        }
        postPropertyChangeEvent(ENDPOINT_STYLE,
                                new Integer(endpointStyle),
                                new Integer(style));
    }

    /**
     * {@inheritDoc}
     */
    public void setIncompleteTimeout(final int timeout) {
        if (timeout < 0) {
            throw new IllegalArgumentException("Invalid incompleteTimeout: "
                                               + timeout);
        }
        postPropertyChangeEvent(INCOMPLETE_TIMEOUT,
                                new Integer(incompleteTimeout),
                                new Integer(timeout));
    }

    /**
     * {@inheritDoc}
     */
    public void setNumResultAlternatives(final int num) {
        postPropertyChangeEvent(NUM_RESULT_ALTERNATIVES,
                                new Integer(this.numResultAlternatives),
                                new Integer(num));
    }

    /**
     * {@inheritDoc}
     */
    public void setSensitivity(final int value) {
        if ((value > MAX_CONFIDENCE)
            || (value < MIN_CONFIDENCE)) {
            throw new IllegalArgumentException("Invalid sensitivity: "
                                               + value);
        }

        postPropertyChangeEvent(SENSITIVITY,
                                new Integer(sensitivity),
                                new Integer(value));
    }


    /**
     * {@inheritDoc}
     */
    public void setSpeedVsAccuracy(final int value) {
        if ((value != MAX_ACCURACY)
            && (value != NORM_ACCURACY)
            && (value != MAX_ACCURACY)) {
            throw new IllegalArgumentException("Invalid speedVsAccuracy: "
                                               + value);
        }
        postPropertyChangeEvent(SPEED_VS_ACCURACY,
                                new Integer(speedVsAccuracy),
                                new Integer(value));
    }

    /**
     * {@inheritDoc}
     */
    public void setResultAudioProvided(final boolean value) {
        postPropertyChangeEvent(RESULT_AUDIO_PROVIDED,
                                new Boolean(resultAudioProvided),
                                new Boolean(value));
    }

    /**
     * {@inheritDoc}
     */
    public void setTrainingProvided(final boolean value) {
        postPropertyChangeEvent(TRAINING_PROVIDED,
                                new Boolean(trainingProvided),
                                new Boolean(value));
    }

    /**
     * {@inheritDoc}
     */
    public boolean setProperty(String propName, Object value) {
        if (propName.equals(TRAINING_PROVIDED)) {
            final Boolean boolVal = (Boolean) value;
            trainingProvided = boolVal.booleanValue();
            return true;
        } else if (propName.equals(RESULT_AUDIO_PROVIDED)) {
            final Boolean boolVal = (Boolean) value;
            resultAudioProvided = boolVal.booleanValue();
            return true;
        } else if (propName.equals(SPEED_VS_ACCURACY)) {
            final Integer intVal = (Integer) value;
            speedVsAccuracy = intVal.intValue();
            return true;
        } else if (propName.equals(SENSITIVITY)) {
            final Integer intVal = (Integer) value;
            sensitivity = intVal.intValue();
            return true;
        } else if (propName.equals(NUM_RESULT_ALTERNATIVES)) {
            final Integer intVal = (Integer) value;
            numResultAlternatives = intVal.intValue();
            return true;
        } else if (propName.equals(INCOMPLETE_TIMEOUT)) {
            final Integer intVal = (Integer) value;
            incompleteTimeout = intVal.intValue();
            return true;
        } else if (propName.equals(ENDPOINT_STYLE)) {
            final Integer intVal = (Integer) value;
            endpointStyle = intVal.intValue();
            return true;
        } else if (propName.equals(CONFIDENCE_THRESHOLD)) {
            final Integer intVal = (Integer) value;
            confidenceThreshold = intVal.intValue();
            return true;
        } else if (propName.equals(COMPLETE_TIMEOUT)) {
            final Integer intVal = (Integer) value;
            completeTimeout = intVal.intValue();
            return true;
        } else if (propName.equals(ADAPTATION)) {
            final Integer intVal = (Integer) value;
            adaptation = intVal.intValue();
            return true;
        }
        return false;
    }
}
