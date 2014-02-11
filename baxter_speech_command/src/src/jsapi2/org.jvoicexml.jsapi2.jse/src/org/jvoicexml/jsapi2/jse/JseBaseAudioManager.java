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

package org.jvoicexml.jsapi2.jse;

import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.speech.AudioException;
import javax.speech.AudioManager;

import org.jvoicexml.jsapi2.BaseAudioManager;
import org.jvoicexml.jsapi2.jse.protocols.JavaSoundParser;

/**
 * Supports the JSAPI 2.0 <code>AudioManager</code>
 * interface.  Actual JSAPI implementations might want to extend
 * or modify this implementation.
 */
public abstract class JseBaseAudioManager extends BaseAudioManager implements AudioManager {
    protected AudioInputStream ais;

    /**
     * Audio format of the audio natively produced by the engine.
     */
    protected AudioFormat engineAudioFormat;

    /** Audio format of that is being received or that is being delivered. */
    protected AudioFormat targetAudioFormat;

    /** Converter from the source (synthesizer) to the target format. */
    private AudioFormatConverter formatConverter;

    /**
     * Constructs a new object.
     * @param format native engine audio format
     */
    public JseBaseAudioManager(final AudioFormat format) {
        engineAudioFormat = format;
        targetAudioFormat = engineAudioFormat;
    }

    /**
     * Retrieves the audio format converter.
     * @return the audio format converter.
     */
    public AudioFormatConverter getAudioFormatConverter() throws IOException {
        if (formatConverter == null) {
            formatConverter = openAudioFormatConverter(engineAudioFormat,
                    targetAudioFormat);
        }
        return formatConverter;
    }

    /**
     * Opens the connection to the configured media locator.
     * @return opened connection
     * @throws IOException
     *         error opening the connection.
     */
    protected URLConnection openURLConnection() throws IOException {
        final String locator = getMediaLocator();
        if (locator == null) {
            return null;
        }

        final URL url;
        try {
            url = new URL(locator);
        } catch (MalformedURLException ex) {
            throw new IllegalArgumentException(ex);
        }

        // Open a connection to URL
        final URLConnection connection = url.openConnection();
        connection.connect();
        return connection;
    }

    /**
     * Opens the audio format converter to convert from the given source
     * format into the given target format.
     * <p>
     * This method must be called in the {@link #audioStart()} method.
     * </p>
     * @param source the source audio format
     * @param target the target audio format.
     * @return the audio format converter.
     * @throws IOException
     *         error opening the format converter
     */
    protected AudioFormatConverter openAudioFormatConverter(
            final AudioFormat source, final AudioFormat target)
        throws IOException {
        return new AudioFormatConverter(this, source, target);
    }

    /**
     * {@inheritDoc}
     */
    public org.jvoicexml.jsapi2.AudioFormat getAudioFormat()
        throws AudioException {
        final String locator = getMediaLocator();
        if (locator != null) {
            //Get matching URI to extract query parameters
            URL url = null;
            try {
                url = new URL(locator);
                AudioFormat format =
                    JavaSoundParser.parse(url);
                return new org.jvoicexml.jsapi2.AudioFormat(
                        format.getEncoding().toString(),
                        format.getSampleRate(), format.getSampleSizeInBits(),
                        format.getChannels(), format.getFrameSize(),
                        format.getFrameRate(), format.isBigEndian());
            } catch (MalformedURLException ex) {
                throw new AudioException(ex.getMessage());
            } catch (URISyntaxException ex) {
                throw new AudioException(ex.getMessage());
            }
        }
        return new org.jvoicexml.jsapi2.AudioFormat(
                engineAudioFormat.getEncoding().toString(),
                engineAudioFormat.getSampleRate(), engineAudioFormat.getSampleSizeInBits(),
                engineAudioFormat.getChannels(), engineAudioFormat.getFrameSize(),
                engineAudioFormat.getFrameRate(), engineAudioFormat.isBigEndian());
    }

    /**
     * Sets the audio format that is being used by this engine.
     * @param audioFormat new audio format.
     */
    public void setEngineAudioFormat(final AudioFormat audioFormat) {
        engineAudioFormat = audioFormat;
    }

    /**
     * Retrieves the audio format that is used by this engine.
     * @return audio format used by this engine.
     */
    public AudioFormat getEngineAudioFormat() {
        return engineAudioFormat;
    }

    /**
     * Retrieves the target audio format.
     * @return target audio format.
     */
    public AudioFormat getTargetAudioFormat() {
        return targetAudioFormat;
    }

    /**
     * {@inheritDoc}
     *
     * Closes the format converter. May be overridden to handle further cleanup.
     */
    @Override
    protected void handleAudioStop() throws AudioException {
        if (formatConverter != null) {
            formatConverter = null;
        }
    }
}

