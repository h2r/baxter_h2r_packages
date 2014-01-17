/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/synthesis/BaseSynthesizerAudioManager.java $
 * Version: $LastChangedRevision: 457 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */
package org.jvoicexml.jsapi2.jse.synthesis;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLConnection;

import javax.sound.sampled.AudioFormat;
import javax.speech.AudioException;
import javax.speech.EngineStateException;

import org.jvoicexml.jsapi2.jse.JseBaseAudioManager;
import org.jvoicexml.jsapi2.jse.protocols.JavaSoundParser;

/**
 * Supports the JSAPI 2.0 <code>AudioManager</code> interface. Actual JSAPI
 * implementations might want to extend or modify this implementation.
 */
public class BaseSynthesizerAudioManager extends JseBaseAudioManager {
    /** The output stream from the synthesizer. */
    private OutputStream outputStream;

    /**
     * Constructs a new object.
     * @param format native engine audio format
     */
    public BaseSynthesizerAudioManager(final AudioFormat format) {
        super(format);
    }

    /**
     * {@inheritDoc}
     */
    public void handleAudioStart() throws AudioException {
        final String locator = getMediaLocator();
        if (locator == null) {
            outputStream = new ClipOutputStream(this);
        } else {
            // Gets IO from that connection if not already present
            if (outputStream == null) {
                // Open URL described in locator
                final URLConnection urlConnection;
                try {
                    urlConnection = openURLConnection();
                    outputStream = urlConnection.getOutputStream();
                } catch (IOException ex) {
                    throw new AudioException("Cannot get OutputStream from URL: "
                            + ex.getMessage());
                }
            }

            try {
                final URL url = new URL(locator);
                targetAudioFormat = JavaSoundParser.parse(url);
            } catch (MalformedURLException e) {
                throw new AudioException(e.getMessage());
            } catch (URISyntaxException e) {
                throw new AudioException(e.getMessage());
            }
        }
        // Configure audio conversions
        try {
            openAudioFormatConverter(engineAudioFormat, targetAudioFormat);
        } catch (IOException e) {
            throw new AudioException(e.getMessage());
        }
    }

    /**
     * {@inheritDoc}
     */
    public void handleAudioStop() throws AudioException {
        if (outputStream != null) {
            try {
                outputStream.close();
            } catch (IOException ex) {
                throw new AudioException(ex.getMessage());
            } finally {
                outputStream = null;
            }
        }
    }

    /**
     * Retrieves the output stream from the synthesizer.
     * @return the output stream.
     */
    public OutputStream getOutputStream() {
        return outputStream;
    }


    /**
     * {@inheritDoc}
     */
    public void setMediaLocator(final String locator, final OutputStream stream)
            throws AudioException {
        super.setMediaLocator(locator);
        this.outputStream = stream;
    }

    /**
     * {@inheritDoc}
     *
     * Throws an {@link IllegalArgumentException} since output streams are not
     * supported.
     */
    public void setMediaLocator(String locator, InputStream stream)
            throws AudioException, EngineStateException,
            IllegalArgumentException, SecurityException {
        throw new IllegalArgumentException("input streams are not supported");
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public InputStream getInputStream() {
        throw new IllegalArgumentException("input streams are not supported");
    }
}
