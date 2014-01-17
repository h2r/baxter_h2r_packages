/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/recognition/BaseRecognizerAudioManager.java $
 * Version: $LastChangedRevision: 589 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: sterad $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007-2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */
package org.jvoicexml.jsapi2.jse.recognition;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;

import javax.sound.sampled.AudioFileFormat;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.UnsupportedAudioFileException;
import javax.speech.AudioException;
import javax.speech.EngineStateException;

import org.jvoicexml.jsapi2.jse.JseBaseAudioManager;
import org.jvoicexml.jsapi2.jse.protocols.JavaSoundParser;

/**
 * Supports the JSAPI 2.0 <code>AudioManager</code> interface. Actual JSAPI
 * implementations might want to extend or modify this implementation.
 */
public class BaseRecognizerAudioManager extends JseBaseAudioManager {
    /** The input stream for the recognizer. */
    private InputStream inputStream;

    /**
     * Constructs a new object.
     * @param format native engine audio format
     */
    public BaseRecognizerAudioManager(final AudioFormat format) {
        super(format);
    }

    /**
     * {@inheritDoc}
     */
    public void handleAudioStart() throws AudioException {
        // Just convert samples 
        if (inputStream instanceof AudioInputStream) {
            AudioInputStream ais = (AudioInputStream)inputStream;
            targetAudioFormat = ais.getFormat();
            inputStream = AudioSystem.getAudioInputStream(engineAudioFormat, ais);
            return;
        }

        final String locator = getMediaLocator();
        // Open URL described in locator
        final InputStream is;
        if (locator == null) {
            targetAudioFormat = getEngineAudioFormat();
            final InputStream source = new LineInputStream(this);
            is = new BufferedInputStream(source);
        } else {
            URL url;
            try {
                url = new URL(locator);
            } catch (MalformedURLException e) {
                throw new AudioException(e.getMessage());
            }
            try {
                final AudioFileFormat format =
                    AudioSystem.getAudioFileFormat(url);
                targetAudioFormat = format.getFormat();
            } catch (UnsupportedAudioFileException e) {
            } catch (IOException e) {
            }
            if (targetAudioFormat == null) {
                try {
                    targetAudioFormat = JavaSoundParser.parse(url);
                } catch (URISyntaxException e) {
                    throw new AudioException(e.getMessage());
                }
            }
            final URLConnection urlConnection;
            try {
                urlConnection = openURLConnection();
            } catch (IOException e) {
                throw new AudioException(e.getMessage());
            }

            try {
                final InputStream source  = urlConnection.getInputStream();
                is = new BufferedInputStream(source);
            } catch (IOException ex) {
                throw new AudioException("Cannot get InputStream from URL: "
                        + ex.getMessage());
            }
        }
        System.err.println(targetAudioFormat);
        final AudioInputStream ais = new AudioInputStream(is,
                    targetAudioFormat, AudioSystem.NOT_SPECIFIED);
        inputStream = AudioSystem.getAudioInputStream(engineAudioFormat, ais);
    }

    /**
     * {@inheritDoc}
     */
    public void handleAudioStop() throws AudioException {
        // Release IO
        if (inputStream != null) {
            try {
                inputStream.close();
            } catch (IOException ex) {
                throw new AudioException(ex.getMessage());
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    public void setMediaLocator(final String locator, final InputStream stream)
            throws AudioException {
        super.setMediaLocator(locator);
        inputStream = stream;
    }

    /**
     * {@inheritDoc}
     *
     * Throws an {@link IllegalArgumentException} since output streams are not
     * supported.
     */
    public void setMediaLocator(String locator, OutputStream stream)
            throws AudioException, EngineStateException,
            IllegalArgumentException, SecurityException {
        throw new IllegalArgumentException("output streams are not supported");
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public InputStream getInputStream() {
        return inputStream;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public OutputStream getOutputStream() {
        throw new IllegalArgumentException("output streams are not supported");
    }
}
