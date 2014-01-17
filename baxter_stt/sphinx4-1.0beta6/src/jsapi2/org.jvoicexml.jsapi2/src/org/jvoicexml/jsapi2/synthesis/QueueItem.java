/*
 * File:    $HeadURL: $
 * Version: $LastChangedRevision: $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: lyncher $
 *
 * JSAPI - An base implementation for JSR 113.
 *
 * Copyright (C) 2007-2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.synthesis;

import javax.speech.synthesis.Speakable;
import javax.speech.synthesis.SpeakableListener;
import javax.speech.AudioSegment;
import javax.speech.synthesis.PhoneInfo;

/**
 * An item of the {@link QueueManager}.
 * @author Renato Cassaca
 * @version $Revision: 1370 $
 */
public class QueueItem {

    private Object source;
    /** A unique id for this queued item. */
    private final int id;

    /** The queued speakable. */
    private final Speakable speakable;

    /** The associated speakable listener. */
    private final SpeakableListener listener;

    /** The associated audio segment. */
    private AudioSegment segment;

    private String[] words;
    private float[] wordsStartTimes;
    private PhoneInfo[] phonesInfo;

    public QueueItem(final int id, final Speakable speakable,
            final SpeakableListener listener) {
        this.id = id;
        this.speakable = speakable;
        this.listener = listener;
        this.segment = null;
        this.words = new String[0];
        this.wordsStartTimes = new float[0];
        this.phonesInfo = new PhoneInfo[0];
        this.source = speakable;
    }

    public QueueItem(int id, Speakable speakable, SpeakableListener listener,
            String text) {
        this(id, speakable, listener);
        this.source = text;
    }

    public QueueItem(int id, AudioSegment audioSegment,
            SpeakableListener listener) {
        this.id = id;
        this.speakable = null;
        this.listener = listener;
        this.segment = audioSegment;
        this.words = new String[0];
        this.wordsStartTimes = new float[0];
        this.phonesInfo = new PhoneInfo[0];
        this.source = audioSegment.getMarkupText();
    }

    /**
     * Retrieves the speakable.
     * @return the speakable.
     */
    public Speakable getSpeakable() {
        return speakable;
    }

    /**
     * Retrieves the speakable listener for this queue item.
     * @return the speakable listener
     */
    public SpeakableListener getListener() {
        return listener;
    }

    /**
     * Retrieves the id of this item.
     * @return id of this queue item.
     */
    public int getId() {
        return id;
    }

    public Object getSource() {
        return source;
    }

    /**
     * Retrieves the associated audio segment.
     * @return the audio segment.
     */
    public AudioSegment getAudioSegment() {
        return segment;
    }

    /**
     * Sets the audio segment.
     * @param audiosegment new value for the audio segment.
     */
    public void setAudioSegment(final AudioSegment audiosegment) {
        segment = audiosegment;
    }

    public String[] getWords() {
        return words;
    }

    public void setWords(String[] w) {
        words = w;
    }

    public float[] getWordsStartTime() {
        return wordsStartTimes;
    }

    public void setWordsStartTimes(float[] starttimes) {
        wordsStartTimes = starttimes;
    }

    public PhoneInfo[] getPhonesInfo() {
        return phonesInfo;
    }

    public void setPhonesInfo(PhoneInfo[] info) {
        phonesInfo = info;
    }
}
