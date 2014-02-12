/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/demo/HelloWorld/src/org/jvoicexml/jsapi2/demo/helloworld/HelloWorldDemo.java $
 * Version: $LastChangedRevision: 593 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - A base implementation for JSR 113.
 *
 * Copyright (C) 2009-2010 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.synthesis;

import java.util.Enumeration;
import java.util.Vector;

import javax.speech.AudioException;
import javax.speech.AudioManager;
import javax.speech.AudioSegment;
import javax.speech.EngineEvent;
import javax.speech.EngineException;
import javax.speech.EngineStateException;
import javax.speech.SpeechEventExecutor;
import javax.speech.synthesis.PhoneInfo;
import javax.speech.synthesis.Speakable;
import javax.speech.synthesis.SpeakableEvent;
import javax.speech.synthesis.SpeakableException;
import javax.speech.synthesis.SpeakableListener;
import javax.speech.synthesis.Synthesizer;
import javax.speech.synthesis.SynthesizerEvent;
import javax.speech.synthesis.SynthesizerListener;
import javax.speech.synthesis.SynthesizerMode;
import javax.speech.synthesis.SynthesizerProperties;

import org.jvoicexml.jsapi2.BaseEngine;
import org.jvoicexml.jsapi2.BaseEngineProperties;


/**
 * Basic synthesizer functions.
 *
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version $Revision: $
 */
public abstract class BaseSynthesizer extends BaseEngine
    implements Synthesizer {
    protected Vector speakableListeners;
    protected SynthesizerProperties synthesizerProperties;
    protected int speakableMask;
    protected final QueueManager queueManager;

    /**
     * Constructs a new object.
     */
    public BaseSynthesizer() {
        this(null);
    }

    /**
     * Constructs a new object.
     * @param engineMode the engine mode for this synthesizer
     */
    public BaseSynthesizer(final SynthesizerMode engineMode) {
        super(engineMode);
        speakableListeners = new Vector();
        final SynthesizerProperties props =
            new BaseSynthesizerProperties(this);
        setSynthesizerProperties(props);
        speakableMask = SpeakableEvent.DEFAULT_MASK;
        setEngineMask(getEngineMask() | SynthesizerEvent.DEFAULT_MASK);
        queueManager = new QueueManager(this);
    }

    /**
     * {@inheritDoc}
     */
    public void fireEvent(final EngineEvent event) {
        synchronized (engineListeners) {
            final SynthesizerEvent synthesizerEvent =
                (SynthesizerEvent) event;
            Enumeration enumeration = engineListeners.elements();
            while (enumeration.hasMoreElements()) {
                final SynthesizerListener listener =
                    (SynthesizerListener) enumeration.nextElement();
                listener.synthesizerUpdate(synthesizerEvent);
            }
        }
    }

    /**
     * postEngineEvent
     *
     * @param oldState long
     * @param newState long
     * @param eventType int
     */
    public void postEngineEvent(long oldState, long newState, int eventType) {
        final SynthesizerEvent event = new SynthesizerEvent(this, eventType,
                oldState, newState, null, false);
        /** @todo Change after adding the queue */
        postEngineEvent(event);
    }

    protected void postSynthesizerEvent(long oldState, long newState,
            int eventType, boolean changedTopOfQueue) {
        switch (eventType){
        case SynthesizerEvent.QUEUE_UPDATED:
        case SynthesizerEvent.QUEUE_EMPTIED:
            break;
        default:
            changedTopOfQueue = false;
        }
        final SynthesizerEvent event = new SynthesizerEvent(this,
                eventType,
                oldState,
                newState,
                null,
                changedTopOfQueue);

        postEngineEvent(event);
    }

    protected void postSpeakableEvent(final SpeakableEvent event,
            final SpeakableListener extraSpeakableListener) {
        if ((getSpeakableMask() & event.getId()) == event.getId()) {
            try {
                final SpeechEventExecutor executor = getSpeechEventExecutor();
                executor.execute(new Runnable() {
                    public void run() {
                        fireSpeakableEvent(event, extraSpeakableListener);
                    }
                });
            } catch (RuntimeException ex) {
                ex.printStackTrace();
            }
        }
    }

    /**
     * Utility function to send a speakable event to all grammar listeners.
     */
    public void fireSpeakableEvent(final SpeakableEvent event,
            final SpeakableListener extraSpeakableListener) {
        if (extraSpeakableListener != null) {
            extraSpeakableListener.speakableUpdate(event);
        }

        if (speakableListeners != null) {
            final Enumeration e = speakableListeners.elements();
            while (e.hasMoreElements()) {
                final SpeakableListener sl =
                    (SpeakableListener) e.nextElement();
                sl.speakableUpdate(event);
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    protected long getEngineStates() {
        return super.getEngineStates() | Synthesizer.QUEUE_EMPTY
        | Synthesizer.QUEUE_NOT_EMPTY;
    }

    /**
     * {@inheritDoc}
     */
    public final void addSpeakableListener(final SpeakableListener listener) {
        if (!speakableListeners.contains(listener)) {
            speakableListeners.addElement(listener);
        }
    }

    /**
     * {@inheritDoc}
     */
    public final void removeSpeakableListener(
            final SpeakableListener listener) {
        speakableListeners.removeElement(listener);
    }

    /**
     * {@inheritDoc}
     */
    public final void addSynthesizerListener(
            final SynthesizerListener listener) {
        addEngineListener(listener);
    }

    /**
     * {@inheritDoc}
     */
    public final void removeSynthesizerListener(
            final SynthesizerListener listener) {
        removeEngineListener(listener);
    }

    /**
     * {@inheritDoc}
     */
    public boolean cancel() throws EngineStateException {
        checkEngineState(DEALLOCATED | DEALLOCATING_RESOURCES);

        // Wait to finalize allocation
        if (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return false;
            }
        }
        return queueManager.cancelItem();
    }

    /**
     * {@inheritDoc}
     */
    public boolean cancel(int id) throws IllegalArgumentException,
            EngineStateException {
        checkEngineState(DEALLOCATED | DEALLOCATING_RESOURCES);

        // Wait to finalize allocation
        while (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return false;
            }
        }
        return queueManager.cancelItem(id);
    }

    public boolean cancelAll() throws EngineStateException {
        checkEngineState(DEALLOCATED | DEALLOCATING_RESOURCES);

        // Wait to finalize allocation
        while (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return false;
            }
        }
        return queueManager.cancelAllItems();
    }

    public String getPhonemes(String text) throws EngineStateException {
        return "";
    }

    public SynthesizerProperties getSynthesizerProperties() {
        return synthesizerProperties;
    }

    public void setSynthesizerProperties(
            SynthesizerProperties synthesizerProperties) {
        this.synthesizerProperties = synthesizerProperties;
        if (synthesizerProperties instanceof BaseEngineProperties) {
            final BaseEngineProperties props =
                (BaseEngineProperties) synthesizerProperties;
            addEnginePropertyChangeRequestListener(props);
        }
    }

    public void setSpeakableMask(int mask) {
        speakableMask = mask;
    }

    public int getSpeakableMask() {
        return speakableMask;
    }

    public int speak(AudioSegment audio, SpeakableListener listener)
        throws EngineStateException, IllegalArgumentException {
        return queueManager.appendItem(audio, listener);
    }


    public int speak(Speakable speakable, SpeakableListener listener)
            throws EngineStateException, SpeakableException,
            IllegalArgumentException {

        // Wait to finalize allocation
        while (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return -1;
            }
        }

        return queueManager.appendItem(speakable, listener);
    }

    /**
     * {@inheritDoc}
     */
    public int speak(final String text, final SpeakableListener listener)
            throws EngineStateException {
        checkEngineState(DEALLOCATED | DEALLOCATING_RESOURCES);

        // Wait to finalize allocation
        while (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                final long delay = 300;
                waitEngineState(ALLOCATED, delay);
            } catch (InterruptedException ex) {
                throw new EngineStateException(
                        "wait engine state interrupted: " + ex.getMessage());
            }
        }

        final Speakable speakable = new BaseSpeakable(text);
        return queueManager.appendItem(speakable, listener, text);
    }

    /**
     * {@inheritDoc}
     */
    public int speakMarkup(final String synthesisMarkup,
            final SpeakableListener listener) throws EngineStateException,
            SpeakableException, IllegalArgumentException {

        // Wait to finalize allocation
        while (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return -1;
            }
        }

        final Speakable speakable = new BaseSpeakable(synthesisMarkup);
        return queueManager.appendItem(speakable, listener);
    }

    /**
     * {@inheritDoc}
     */
    protected final void baseAllocate() throws EngineStateException,
            EngineException, AudioException, SecurityException {

        // Proceed to real engine allocation
        handleAllocate();
        long[] states = setEngineState(CLEAR_ALL_STATE, ALLOCATED | PAUSED
                | DEFOCUSED | QUEUE_EMPTY);

        // Starts AudioManager
        final AudioManager audioManager = getAudioManager();
        audioManager.audioStart();

        postEngineEvent(states[0], states[1], EngineEvent.ENGINE_ALLOCATED);
    }

    /**
     * Perform the real allocation of the synthesizer.
     * @throws AudioException
     *          if any audio request fails 
     * @throws EngineException
     *          if an allocation error occurred or the Engine is not
     *          operational. 
     * @throws EngineStateException
     *          if called for an Engine in the DEALLOCATING_RESOURCES state 
     * @throws SecurityException
     *          if the application does not have permission for this Engine
     */
    protected abstract void handleAllocate() throws EngineStateException,
        EngineException, AudioException, SecurityException;


    /**
     * {@inheritDoc}
     */
    protected void baseDeallocate() throws EngineStateException,
            EngineException, AudioException {

        // Stops AudioManager
        final AudioManager audioManager = getAudioManager();
        audioManager.audioStop();

        // Procceed to real engine deallocation
        handleDeallocate();
        
        // Adapt the state
        long[] states = setEngineState(CLEAR_ALL_STATE, DEALLOCATED);
        postEngineEvent(states[0], states[1],
                EngineEvent.ENGINE_DEALLOCATED);
    }

    /**
     * {@inheritDoc}
     */
    protected void basePause() {
        handlePause();
    }

    /**
     * Called from the <code>resume</code> method. Override in subclasses.
     *
     * @todo Handle grammar updates
     */
    protected boolean baseResume() {
        return handleResume();
    }

    abstract protected void handleDeallocate() throws EngineStateException,
        EngineException, AudioException;

    abstract protected void handlePause();

    abstract protected boolean handleResume();

    /**
     * Speak the item with the given id.
     * <p>
     * Implementations synthesize the text to speak and make it available
     * via an {@link java.io.InputStream}. This input stream is used to
     * create a {@link javax.speech.AudioSegment} which is passed to the
     * {@link QueueManager} to be played back using the settings from the
     * {@link javax.speech.AudioManager}.
     * </p>
     * @param id id of the text to speak
     * @param item the text to speak
     */
    protected abstract void handleSpeak(final int id, final String item);

    /**
     * Speak the SSML item with the given id.
     * <p>
     * Implementations synthesize the text to speak and make it available
     * via an {@link java.io.InputStream}. This input stream is used to
     * create a {@link javax.speech.AudioSegment} which is passed to the
     * {@link QueueManager} to be played back using the settings from the
     * {@link javax.speech.AudioManager}.
     * </p>
     * @param id id of the SSML markup text to speak
     * @param item the SSML markup text to speak
     */
    protected abstract void handleSpeak(final int id, final Speakable item);

    /**
     * Returns a <code>String</code> of the names of all the
     * <code>Engine</code> states in the given <code>Engine</code> state.
     *
     * @param state
     *                the bitmask of states
     *
     * @return a <code>String</code> containing the names of all the states
     *         set in <code>state</code>
     */
    public String stateToString(final long state) {
        final String stateString = super.stateToString(state);
        final StringBuffer buf = new StringBuffer(stateString);
        if ((state & Synthesizer.QUEUE_EMPTY) != 0) {
            buf.append(" QUEUE_EMPTY ");
        }
        if ((state & Synthesizer.QUEUE_NOT_EMPTY) != 0) {
            buf.append(" QUEUE_NOT_EMPTY ");
        }
        return buf.toString();
    }

    /**
     * Cancels the item that is currently being played back.
     * @return <code>true</code> if the item was canceled
     * @exception EngineStateException
     *            if the engine was in an invalid state
     */
    protected abstract boolean handleCancel() throws EngineStateException;

    /**
     * Cancels all items in the play queue.
     * @return <code>true</code> if at least one item was canceled
     * @exception EngineStateException
     *            if the engine was in an invalid state
     */
    protected abstract boolean handleCancelAll() throws EngineStateException;

    /**
     * Cancels the item with the given id.
     * @param id id of the item to cancel
     * @return <code>true</code> if the item with the given id was canceled.
     * @exception EngineStateException
     *            if the engine was in an invalid state
     */
    protected abstract boolean handleCancel(final int id)
        throws EngineStateException;

    /**
     * Sets AudioSegment in a queueItem.
     *
     * @param id current id
     * @param audioSegment the audio segment.
     */
    protected void setAudioSegment(final int id,
            final AudioSegment audioSegment) {
        queueManager.setAudioSegment(id, audioSegment);
    }

    /**
     * Utility method to set words in a queue item.
     *
     * @param itemId the id of the queued item
     * @param String[] words
     */
    protected void setWords(int itemId, String[] words) {
        queueManager.setWords(itemId, words);
    }

    /**
     * Set words times in a queueItem (Not JSAPI2)
     *
     * @param itemId int
     * @param float[] words
     */
    protected void setWordsStartTimes(int itemId, float[] starttimes) {
        queueManager.setWordsStartTimes(itemId, starttimes);
    }

    protected void setPhonesInfo(int itemId, PhoneInfo[] phonesinfo) {
        queueManager.setPhonesInfo(itemId, phonesinfo);
    }

    /**
     * Retrieves the queue manager.
     * @return the queue manager
     */
    protected QueueManager getQueueManager() {
        return queueManager;
    }


}
