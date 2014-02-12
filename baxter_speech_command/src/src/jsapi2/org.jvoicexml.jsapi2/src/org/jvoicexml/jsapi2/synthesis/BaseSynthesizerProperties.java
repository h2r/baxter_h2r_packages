/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2/src/org/jvoicexml/jsapi2/synthesis/BaseSynthesizerProperties.java $
 * Version: $LastChangedRevision: 295 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007-2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.synthesis;

import javax.speech.Engine;
import javax.speech.synthesis.Synthesizer;
import javax.speech.synthesis.SynthesizerMode;
import javax.speech.synthesis.SynthesizerProperties;
import javax.speech.synthesis.Voice;

import org.jvoicexml.jsapi2.BaseEngineProperties;

/**
 * Base implementation of {@link SynthesizerProperties}.
 *
 * <p>
 * Actual JSAPI2 implementations may want to override this class to
 * apply the settings to the synthesizer.
 * </p>
 *
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version $Revision: 295 $
 */
public class BaseSynthesizerProperties extends BaseEngineProperties
    implements SynthesizerProperties {
    /** Name of the interruptibility property in events. */
    public static final String INTERRUPTIBILITY = "interruptibility";

    /** Name of the pitch property in events. */
    public static final String PITCH = "pitch";

    /** Name of the pitch range property in events. */
    public static final String PITCH_RANGE = "pitchRange";

    /** Name of the speaking rate property in events. */
    public static final String SPEAKING_RATE = "speakingRate";

    /** Name of the speaking rate property in events. */
    public static final String VOICE = "voice";

    /** Name of the volume property in events. */
    public static final String VOLUME = "volume";

    private int interruptibility;

    private int pitch;

    private int pitchRange;

    private int speakingRate;

    private Voice voice;

    private int volume;

    /**
     * Constructs a new Object.
     * @param synthesizer reference to the synthesizer.
     */
    public BaseSynthesizerProperties(final Synthesizer synthesizer) {
        super(synthesizer);

        interruptibility = OBJECT_LEVEL;
        pitch = 160;
        pitchRange = (int)(160 * 0.60);
        speakingRate = DEFAULT_RATE;
        volume = MEDIUM_VOLUME;
        //Set default voice
        final Engine engine = getEngine();
        final SynthesizerMode mode = (SynthesizerMode) engine.getEngineMode();
        if (mode == null) {
            voice = null;
        } else {
            Voice[] voices = mode.getVoices();
            if ((voices != null) && (voices.length > 0)) {
                voice = voices[0];
            } else {
                voice = null;
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    public int getInterruptibility() {
        return interruptibility;
    }

    /**
     * {@inheritDoc}
     */
    public void setInterruptibility(final int level) {
        if ((level != WORD_LEVEL) && (level != OBJECT_LEVEL)
                && (level != QUEUE_LEVEL)) {
            throw new IllegalArgumentException("Invalid interruptibiliy level :"
                    + level);
        }
        postPropertyChangeRequestEvent(INTERRUPTIBILITY,
                new Integer(this.interruptibility),
                new Integer(level));
    }

    /**
     * {@inheritDoc}
     */
    public int getPitch() {
        return pitch;
    }

    /**
     * {@inheritDoc}
     */
    public void setPitch(final int hertz) {
        if (hertz <= 0) {
            throw new IllegalArgumentException(
                    "Pitch is not a positive integer:"  + hertz);
        }
        postPropertyChangeRequestEvent(PITCH,
                new Integer(this.pitch), new Integer(hertz));
    }

    /**
     * {@inheritDoc}
     */
    public int getPitchRange() {
        return pitchRange;
    }

    /**
     * {@inheritDoc}
     */
    public void setPitchRange(final int hertz) {
        if (hertz < 0) {
            throw new IllegalArgumentException(
                    "Pitch is a negative integer:"  + hertz);
        }
        postPropertyChangeRequestEvent(PITCH_RANGE,
                new Integer(this.pitchRange), new Integer(hertz));
    }

    /**
     * {@inheritDoc}
     */
    public int getSpeakingRate() {
        return speakingRate;
    }

    /**
     * {@inheritDoc}
     */
    public void setSpeakingRate(final int wpm) {
        if (wpm < 0) {
            throw new IllegalArgumentException(
                    "Speaking rate is not a postivie integer:"  + wpm);
        }
        postPropertyChangeRequestEvent(SPEAKING_RATE,
                new Integer(this.speakingRate), new Integer(wpm));
    }

    /**
     * {@inheritDoc}
     */
    public Voice getVoice() {
        return voice;
    }

    /**
     * {@inheritDoc}
     */
    public void setVoice(final Voice voice) {
        final Engine synthesizer = getEngine();
        final SynthesizerMode mode =
            (SynthesizerMode) synthesizer.getEngineMode();
        if (mode == null) {
            return;
        }
        final Voice[] voices = mode.getVoices();
        for (int i = 0; i < voices.length; i++) {
            final Voice current = voices[i];
            if (current.match(voice)) {
                postPropertyChangeRequestEvent(VOICE,
                        this.voice, voice);
                return;
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    public int getVolume() {
        return volume;
    }


    /**
     * {@inheritDoc}
     */
    public void setVolume(final int volume) {
        if ((volume < MIN_VOLUME) || (volume > MAX_VOLUME)) {
            throw new IllegalArgumentException("Volume is out of range: "
                    + volume);
        }
        postPropertyChangeRequestEvent(VOLUME,
                new Integer(this.volume), new Integer(volume));
    }

    /**
     * {@inheritDoc}
     */
    public void reset() {
        setInterruptibility(OBJECT_LEVEL);
        setPitch(160);
        setPitchRange((int)(160 * 0.60));
        setSpeakingRate(DEFAULT_RATE);
        setVolume(MEDIUM_VOLUME);
        //Set default voice
        final Engine engine = getEngine();
        final SynthesizerMode mode = (SynthesizerMode) engine.getEngineMode();
        if (mode == null) {
            setVoice(null);
        } else {
            Voice[] voices = mode.getVoices();
            if ((voices != null) && (voices.length > 0)) {
                setVoice(voices[0]);
            } else {
                setVoice(null);
            }
        }

        super.reset();
    }

    /**
     * {@inheritDoc}
     */
    protected boolean setProperty(String propName, Object value) {
        if (propName.equals(INTERRUPTIBILITY)) {
            final Integer intVal = (Integer) value;
            interruptibility = intVal.intValue();
            return true;
        } else if (propName.equals(PITCH)) {
            final Integer intVal = (Integer) value;
            pitch = intVal.intValue();
            return true;
        } else if (propName.equals(PITCH_RANGE)) {
            final Integer intVal = (Integer) value;
            pitchRange = intVal.intValue();
            return true;
        } else if (propName.equals(SPEAKING_RATE)) {
            final Integer intVal = (Integer) value;
            speakingRate = intVal.intValue();
            return true;
        } else if (propName.equals(VOICE)) {
            voice = (Voice) value;
            return true;
        } else if (propName.equals(VOLUME)) {
            final Integer intVal = (Integer) value;
            volume = intVal.intValue();
            return true;
        }
        return false;
    }

}
