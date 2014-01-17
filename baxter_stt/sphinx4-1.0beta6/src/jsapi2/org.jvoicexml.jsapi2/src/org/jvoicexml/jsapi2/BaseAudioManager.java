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

package org.jvoicexml.jsapi2;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.Vector;

import javax.speech.AudioEvent;
import javax.speech.AudioException;
import javax.speech.AudioListener;
import javax.speech.AudioManager;
import javax.speech.Engine;
import javax.speech.EngineStateException;
import javax.speech.SpeechEventExecutor;

/**
 * Supports the JSAPI 2.0 <code>AudioManager</code>
 * interface.  Actual JSAPI implementations might want to extend
 * or modify this implementation.
 */
public abstract class BaseAudioManager implements AudioManager {
    /**
     * List of <code>AudioListeners</code> registered for
     * <code>AudioEvents</code> on this object.
     */
    private Vector audioListeners;

    /**  Mask to filter events. */ 
    private int audioMask;

    /** The media locator. */
    private String mediaLocator;
    
    /** The associated engine. */
    private Engine engine;

    /**
     * Class constructor.
     */
    public BaseAudioManager() {
        audioListeners = new Vector();
        audioMask = AudioEvent.DEFAULT_MASK;
    }

    /**
     * Sets the engine.
     * @param value the engine.
     */
    public final void setEngine(final Engine value) {
        engine = value;
    }

    /**
     * Requests notification of <code>AudioEvents</code> from the
     * <code>AudioManager</code>.
     *
     * @param listener the listener to add
     */
    public final void addAudioListener(final AudioListener listener) {
        synchronized (audioListeners) {
            if (!audioListeners.contains(listener)) {
                audioListeners.addElement(listener);
            }
        }
    }

    /**
     * Removes an <code>AudioListener</code> from the list of
     * <code>AudioListeners</code>.
     *
     * @param listener the listener to remove
     */
    public final void removeAudioListener(final AudioListener listener) {
        synchronized (audioListeners) {
            audioListeners.removeElement(listener);
        }
    }

    /**
     * {@inheritDoc}
     */
    public final int getAudioMask() {
        return audioMask;
    }

    /**
     * {@inheritDoc}
     */
    public final void setAudioMask(final int mask) {
        audioMask = mask;
    }

    /**
     * {@inheritDoc}
     */
    public final void audioStart() throws SecurityException,
            AudioException, EngineStateException {
        final String locator = getMediaLocator();
        if ((locator != null) && !isSupportsAudioManagement()) {
            throw new SecurityException(
                    "AudioManager has no permission to access audio resources");
        }

        handleAudioStart();

        final AudioEvent event = new AudioEvent(engine,
                AudioEvent.AUDIO_STARTED);
        postAudioEvent(event);
    }

    /**
     * Handles further processing if the audio output has to be started by
     * a call to {@link #audioStart()}.
     * @throws AudioException
     *         error stopping
     */
    protected abstract void handleAudioStart() throws AudioException;

    /**
     * {@inheritDoc}
     */
    public final void audioStop() throws SecurityException,
            AudioException, EngineStateException {
        if (!isSupportsAudioManagement()) {
            throw new SecurityException(
                    "AudioManager has no permission to access audio resources");
        }
        
        if (!(engine.testEngineState(Engine.PAUSED) || engine.testEngineState(Engine.DEALLOCATING_RESOURCES))) {
            throw new EngineStateException(
                    "The Engine has not been paused");
        }

        handleAudioStop();

        final AudioEvent event = new AudioEvent(engine,
                AudioEvent.AUDIO_STOPPED);
        postAudioEvent(event);
    }

    /**
     * Handles further processing if the audio output has to be stopped by
     * a call to {@link #audioStop()}.
     * @throws AudioException
     *         error stopping
     */
    protected abstract void handleAudioStop() throws AudioException;

    /**
     * {@inheritDoc}
     */
    public final void setMediaLocator(final String locator)
        throws AudioException, EngineStateException, IllegalArgumentException,
            SecurityException {

        if (!isSupportsAudioManagement()) {
            throw new SecurityException(
                    "AudioManager has no permission to access audio resources");
        }

        // Ensure that media locator is supported
        if (!isSupportedMediaLocator(locator)) {
            throw new AudioException("Unsupported locator: " + locator);
        }

        mediaLocator = locator;
        
        final AudioEvent event = new AudioEvent(engine,
                AudioEvent.AUDIO_CHANGED);
        postAudioEvent(event);
    }

    /**
     * {@inheritDoc}
     */
    public final String getMediaLocator() {
        return mediaLocator;
    }

    /**
     * {@inheritDoc}
     * @todo This is just a dummy implementation
     */
    public String[] getSupportedMediaLocators(final String locator)
        throws IllegalArgumentException {
        return new String[] {mediaLocator};
    }

    /**
     * {@inheritDoc}
     */
    public final boolean isSupportedMediaLocator(final String locator)
        throws IllegalArgumentException {
        final String[] supportedMediaLocators = getSupportedMediaLocators(
                locator);
        return supportedMediaLocators != null;
    }

    /**
     * Checks if audio management is supported.
     * @return <code>true</code> if audio management is supported.
     */
    protected final boolean isSupportsAudioManagement() {
        final String management =
            System.getProperty("javax.speech.supports.audio.management");
        if (management == null) {
            return false;
        }
        return management.equals("true");
    }

    /**
     * {@inheritDoc}
     *
     * This implementation checks only for equal media locators.
     */
    public boolean isSameChannel(final AudioManager audioManager) {
        if (audioManager == null) {
            return false;
        }
        final String otherLocator = audioManager.getMediaLocator();
        if (otherLocator == null) {
            return mediaLocator == null;
        }
        if (mediaLocator == null) {
            return false;
        }
        return mediaLocator.equals(otherLocator);
    }

    
    /**
     * Notifies all listeners about the audio event using the configured
     * {@link javax.speech.SpeechEventExecutor}.
     * @param event the event to notify.
     */
    protected final void postAudioEvent(final AudioEvent event) {
        final int eventId = event.getId();
        if ((getAudioMask() & eventId) == eventId) {

            final Runnable runnable = new Runnable() {
                public void run() {
                    synchronized (audioListeners) {
                        final Enumeration enumeration =
                            audioListeners.elements();
                        while (enumeration.hasMoreElements()) {
                            final AudioListener listener =
                                (AudioListener) enumeration.nextElement();
                            listener.audioUpdate(event);
                        }
                    }
                }
            };

            try {
                final SpeechEventExecutor executor =
                    engine.getSpeechEventExecutor();
                executor.execute(runnable);
            } catch (RuntimeException ex) {
                //Ignore exception
                ex.printStackTrace();
            }
        }
    }

    /**
     * Retrieves the output stream associated with the given media locator.
     * @return output stream, <code>null</code> if streaming is not supported.
     */
    public abstract OutputStream getOutputStream();

    /**
     * Retrieves the input stream associated with the given media locator.
     * @return input stream, <code>null</code> if streaming is not supported.
     */
    public abstract InputStream getInputStream();

    /**
     * Retrieves the used audio format.
     * @return the used audio format
     * @throws AudioException the audio format could not be determined
     */
    public abstract AudioFormat getAudioFormat() throws AudioException;
}

