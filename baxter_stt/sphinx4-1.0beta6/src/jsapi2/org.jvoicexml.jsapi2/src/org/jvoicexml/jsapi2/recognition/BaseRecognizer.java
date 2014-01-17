/*
 * File:    $HeadURL: $
 * Version: $LastChangedRevision:  $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: lyncher $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 * This class is based on work by SUN Microsystems and
 * Carnegie Mellon University
 *
 * Copyright 1998-2003 Sun Microsystems, Inc.
 *
 * Portions Copyright 2001-2004 Sun Microsystems, Inc.
 * Portions Copyright 1999-2001 Language Technologies Institute,
 * Carnegie Mellon University.
 * All Rights Reserved.  Use is subject to license terms.
 *
 * Permission is hereby granted, free of charge, to use and distribute
 * this software and its documentation without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of this work, and to
 * permit persons to whom this work is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 * 2. Any modifications must be clearly marked as such.
 * 3. Original authors' names are not deleted.
 * 4. The authors' names are not used to endorse or promote products
 *    derived from this software without specific prior written
 *   permission.
 *
 * SUN MICROSYSTEMS, INC., CARNEGIE MELLON UNIVERSITY AND THE
 * CONTRIBUTORS TO THIS WORK DISCLAIM ALL WARRANTIES WITH REGARD TO THIS
 * SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS, IN NO EVENT SHALL SUN MICROSYSTEMS, INC., CARNEGIE MELLON
 * UNIVERSITY NOR THE CONTRIBUTORS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 * OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */
package org.jvoicexml.jsapi2.recognition;

import java.util.Enumeration;
import java.util.Vector;

import javax.speech.AudioException;
import javax.speech.AudioManager;
import javax.speech.EngineEvent;
import javax.speech.EngineException;
import javax.speech.EngineStateException;
import javax.speech.SpeechEventExecutor;
import javax.speech.VocabularyManager;
import javax.speech.recognition.Grammar;
import javax.speech.recognition.GrammarException;
import javax.speech.recognition.GrammarManager;
import javax.speech.recognition.Recognizer;
import javax.speech.recognition.RecognizerEvent;
import javax.speech.recognition.RecognizerListener;
import javax.speech.recognition.RecognizerMode;
import javax.speech.recognition.RecognizerProperties;
import javax.speech.recognition.Result;
import javax.speech.recognition.ResultEvent;
import javax.speech.recognition.ResultListener;
import javax.speech.recognition.RuleGrammar;
import javax.speech.recognition.SpeakerManager;

import org.jvoicexml.jsapi2.BaseAudioManager;
import org.jvoicexml.jsapi2.BaseEngine;
import org.jvoicexml.jsapi2.BaseEngineProperties;
import org.jvoicexml.jsapi2.BaseVocabularyManager;


/**
 * Skeletal Implementation of the JSAPI Recognizer interface.
 *
 * This class is useful by itself for debugging, e.g. you
 * can load grammars and simulate a recognizer recognizing
 * some text, etc.
 *
 * <P>
 *
 * Actual JSAPI recognizer implementations might want to extend or
 * modify this implementation.
 *
 */
public abstract class BaseRecognizer extends BaseEngine implements Recognizer {
    protected Vector resultListeners;

    protected boolean hasModalGrammars;

    protected boolean supportsNULL = true;
    protected boolean supportsVOID = true;

    // used when printing grammars
    public RuleGrammar currentGrammar;

    // Set to true if recognizer cannot handle partial grammar loading.
    protected boolean reloadAll;

    private final SpeakerManager speakerManager;
    protected RecognizerProperties recognizerProperties;
    private int resultMask;

    /** The related grammar manager. */
    private final GrammarManager grammarManager;

    protected final Vector uncommitedDeletedGrammars;

    /**
     * Create a new Recognizer in the DEALLOCATED state.
     */
    public BaseRecognizer() {
        this(null);
    }

    /**
     * Create a new Recognizer in the DEALLOCATED state.
     * @param mode the recognizer mode
     */
    public BaseRecognizer(final RecognizerMode mode) {
        super(mode);
        uncommitedDeletedGrammars = new Vector();
        final BaseAudioManager audioManager =
            (BaseAudioManager) getAudioManager();
        audioManager.setEngine(this);
        resultListeners = new Vector();
        speakerManager = new BaseSpeakerManager();
        final RecognizerProperties props =
            new BaseRecognizerProperties(this);
        setRecognizerProperties(props);
        grammarManager = createGrammarManager();
        resultMask = ResultEvent.DEFAULT_MASK;
        setEngineMask(getEngineMask() | RecognizerEvent.DEFAULT_MASK);
    }

    /**
     * Creates a new grammar manager.
     * @return the created grammar manager
     */
    protected abstract GrammarManager createGrammarManager();

    /**
     * {@inheritDoc}
     */
    protected VocabularyManager createVocabularyManager() {
        return new BaseVocabularyManager();
    }

    /**
     * {@inheritDoc}
     */
    public GrammarManager getGrammarManager() {
        return grammarManager;
    }

    /**
     * {@inheritDoc}
     */
    public final void requestFocus() throws EngineStateException {
        checkEngineState(DEALLOCATED | DEALLOCATING_RESOURCES);

        if (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return;
            }
        }

        // Do nothing if the state is already OK
        if (testEngineState(FOCUSED)) {
            return;
        }

        long[] states = setEngineState(DEFOCUSED, FOCUSED);
        postEngineEvent(states[0], states[1], EngineEvent.ENGINE_FOCUSED);

        notifyGrammarActivation();
        handleRequestFocus();
    }

    /**
     * {@inheritDoc}
     */
    public final void releaseFocus() throws EngineStateException {
        checkEngineState(DEALLOCATED | DEALLOCATING_RESOURCES);

        if (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return;
            }
        }

        if (testEngineState(DEFOCUSED)) {
            return;
        }

        long[] states = setEngineState(FOCUSED, DEFOCUSED);
        postEngineEvent(states[0], states[1], EngineEvent.ENGINE_DEFOCUSED);

        notifyGrammarActivation();
        handleReleaseFocus();
    }

    /**
     * {@inheritDoc}
     */
    public final void pause(final int mode) throws EngineStateException {
        // Validate current state
        if (testEngineState(PAUSED)) {
            return;
        }

        checkEngineState(DEALLOCATED | DEALLOCATING_RESOURCES);

        if (testEngineState(ALLOCATING_RESOURCES)) {
            try {
                waitEngineState(ALLOCATED);
            } catch (InterruptedException ex) {
                return;
            }
        }

        // Handle pause
        basePause(mode);
        long[] states = setEngineState(RESUMED, PAUSED);
        postEngineEvent(states[0], states[1], EngineEvent.ENGINE_PAUSED);
    }

    /**
     * Notify any grammars if their activation state has been changed.
     */
    protected void notifyGrammarActivation() {
        /*if (grammars == null) {
            return;
        }*/
        /*  Enumeration e = grammars.elements();
          while (e.hasMoreElements()) {
              RuleGrammar rg = (RuleGrammar) e.nextElement();
              boolean active = isActive(rg);*/
        /*     if (active != rg.isActive()) {
                 rg.grammarActive = active;
                 if (active) {
                     rg.postGrammarActivated();
                 } else {
                     rg.postGrammarDeactivated();
                 }
             }*/
        // }
    }


    /**
     * {@inheritDoc}
     */
    public void fireEvent(final EngineEvent event) {
        synchronized (engineListeners) {
            final RecognizerEvent recognizerEvent =
                (RecognizerEvent) event;
            Enumeration enumeration = engineListeners.elements();
            while (enumeration.hasMoreElements()) {
                final RecognizerListener listener =
                    (RecognizerListener) enumeration.nextElement();
                listener.recognizerUpdate(recognizerEvent);
            }
        }
    }


    public void postEngineEvent(long oldState, long newState, int eventType) {
        switch (eventType) {
        case RecognizerEvent.SPEECH_STARTED:
        case RecognizerEvent.SPEECH_STOPPED:
        case RecognizerEvent.RECOGNIZER_BUFFERING:
        case RecognizerEvent.RECOGNIZER_NOT_BUFFERING:
            postEngineEvent(oldState, newState, eventType, 0); /** @todo DGMR rever este 0; nao faltara o audioposition nos parametros da funcao? o speechstart, o stop o buffering e not buferring passam por aqui? */
            break;
        default:
            postEngineEvent(oldState, newState, eventType,
                            RecognizerEvent.UNKNOWN_AUDIO_POSITION);
        }
    }

    public void postEngineEvent(long oldState, long newState, int eventType,
                                long audioPosition) {
        switch (eventType) {
        case RecognizerEvent.SPEECH_STARTED:
        case RecognizerEvent.SPEECH_STOPPED:
        case RecognizerEvent.RECOGNIZER_BUFFERING:
        case RecognizerEvent.RECOGNIZER_NOT_BUFFERING:
            break;
        default:
            audioPosition = RecognizerEvent.UNKNOWN_AUDIO_POSITION;
        }

        final RecognizerEvent event = new RecognizerEvent(this,
                eventType,
                oldState,
                newState,
                null,
                null,
                audioPosition);
        postEngineEvent(event);
    }

    protected void postResultEvent(final ResultEvent event) {
        final SpeechEventExecutor executor = getSpeechEventExecutor();
        try {
            executor.execute(new Runnable() {
                public void run() {
                    fireResultEvent(event);
                }
            });
        } catch (RuntimeException e) {
            e.printStackTrace();
        }
        final Result result = (Result) event.getSource();
        postResultEvent(result, event, executor);
    }

    /**
     * Notification of the result to the registered {@link ResultListener}s
     * about the given result event.
     * @param result the result
     * @param event the event
     * @param executor the speech event executor
     */
    protected abstract void postResultEvent(Result result, ResultEvent event,
            SpeechEventExecutor executor);

    public void fireResultEvent(final ResultEvent event) {
        Enumeration listeners = resultListeners.elements();
        while (listeners.hasMoreElements()) {
            ResultListener el = (ResultListener) listeners.nextElement();
            ((ResultListener) el).resultUpdate((ResultEvent) event);
        }
    }


    /**
     * {@inheritDoc}
     */
    public void addRecognizerListener(final RecognizerListener listener) {
        addEngineListener(listener);
    }

    /**
     * {@inheritDoc}
     */
    public void removeRecognizerListener(final RecognizerListener listener) {
        removeEngineListener(listener);
    }


    /**
     * Request notification of Result events from the Recognizer.
     * From javax.speech.recognition.Recognizer.
     * @param listener the listener to add.
     */
    public void addResultListener(final ResultListener listener) {
        if (!resultListeners.contains(listener)) {
            resultListeners.addElement(listener);
        }
    }

    /**
     * Remove a ResultListener from the list of ResultListeners.
     * From javax.speech.recognition.Recognizer.
     * @param listener the listener to remove.
     */
    public void removeResultListener(ResultListener listener) {
        resultListeners.removeElement(listener);
    }

    /**
     * Get the RecognizerProperties of this Recognizer.
     * From javax.speech.recognition.Recognizer.
     */
    public RecognizerProperties getRecognizerProperties() {
        return recognizerProperties;
    }

    /**
     * Sets the properties for this recognizer.
     * @param properties the properties
     */
    protected void setRecognizerProperties(RecognizerProperties
                                        properties) {
        recognizerProperties = properties;
        if (properties instanceof BaseEngineProperties) {
            final BaseEngineProperties props =
                (BaseEngineProperties) properties;
            addEnginePropertyChangeRequestListener(props);
        }
    }

    /**
     * {@inheritDoc}
     */
    public SpeakerManager getSpeakerManager() {
        return speakerManager;
    }

    /**
     * {@inheritDoc}
     */
    public void setResultMask(final int mask) {
        resultMask = mask;
    }

    /**
     * {@inheritDoc}
     */
    public int getResultMask() {
        return resultMask;
    }


    /**
     *
     * @throws EngineStateException
     */
    public void processGrammars() throws EngineStateException {

        //Flag that indicates if grammars were changed
        boolean existChanges = false;

        //Build a new grammar set, with all enabled grammars
        Vector newGrammars = new Vector();

        //Commit all grammars pending changes
        final Grammar[] grammars = grammarManager.listGrammars();
        for (int i = 0; i < grammars.length; i++) {
            final Grammar grammar = grammars[i];
            boolean updated;
            try {
                updated = processGrammar(grammar);
            } catch (GrammarException e) {
                updated = false;
                e.printStackTrace();
            }
            // Update "modified-flag"
            existChanges = existChanges || updated;
            if (grammar.isActivatable()) {
                final GrammarDefinition definition =
                    new GrammarDefinition(grammar.toString(),
                            grammar.getReference(), existChanges);
                newGrammars.addElement(definition);
            }
        }

        // Set grammars
        boolean setGrammarsResult = setGrammars(newGrammars);

        //Raise proper events
        if (existChanges) {
            if (setGrammarsResult) {
                postEngineEvent(PAUSED, RESUMED,
                        RecognizerEvent.CHANGES_COMMITTED);
                for (int i = 0; i < grammars.length; i++) {
                    final BaseGrammar baseGrammar = (BaseGrammar) grammars[i];
                    baseGrammar.postGrammarChangesCommitted();
                }
            } else {
                for (int i = 0; i < grammars.length; i++) {
                    final BaseGrammar baseGrammar = (BaseGrammar) grammars[i];
                    baseGrammar.postGrammarChangesRejected();
                }
            }
        }
    }

    /**
     * Processes the given grammar.
     * @param grammar the grammar to process.
     * @exception GrammarException error processing the grammar
     * @return <code>true</code> if the grammar has been updated
     */
    protected abstract boolean processGrammar(final Grammar grammar)
        throws GrammarException;

    /**
     * {@inheritDoc}
     */
    protected long getEngineStates() {
        return super.getEngineStates() | Recognizer.LISTENING
            | Recognizer.PROCESSING;
    }


    /**
     * {@inheritDoc}
     */
    public String stateToString(final long state) {
        StringBuffer buf = new StringBuffer(super.stateToString(state));
        if ((state & Recognizer.LISTENING) != 0) {
            buf.append(" LISTENING ");
        }
        if ((state & Recognizer.PROCESSING) != 0) {
            buf.append(" PROCESSING ");
        }
        return buf.toString();
    }


    /**
     * Called from the <code>allocate</code> method.
     *
     * @see #allocate
     *
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
    protected final void baseAllocate() throws EngineStateException,
            EngineException, AudioException, SecurityException {
        final AudioManager audioManager = getAudioManager();
        audioManager.audioStart();

        // Proceed to real engine allocation
        handleAllocate();
        long[] states = setEngineState(CLEAR_ALL_STATE,
                ALLOCATED | PAUSED | DEFOCUSED |
                NOT_BUFFERING);
        postEngineEvent(states[0], states[1], EngineEvent.ENGINE_ALLOCATED);
    }

    /**
     * Perform the real allocation of the recognizer.
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
     * Called from the <code>deallocate</code> method.  Override this in
     * subclasses.
     *
     * @throws EngineException if this <code>Engine</code> cannot be
     *   deallocated.
     */
    protected final void baseDeallocate() throws EngineStateException,
            EngineException, AudioException {

        // Procceed to real engine deallocation
        try {
            handleDeallocate();
        } finally {
            // Stops AudioManager
            final AudioManager audioManager = getAudioManager();
            audioManager.audioStop();
        }
        long[] states = setEngineState(CLEAR_ALL_STATE, DEALLOCATED);
        postEngineEvent(states[0], states[1],
                EngineEvent.ENGINE_DEALLOCATED);
    }

    /**
     * {@inheritDoc}
     */
    protected final void basePause() {
        handlePause();
        setEngineState(LISTENING | PROCESSING,
                       getEngineState() & ~LISTENING & ~PROCESSING);
        setEngineState(BUFFERING, NOT_BUFFERING);
    }

    /**
     * {@inheritDoc}
     */
    protected final void basePause(final int flags) {
        handlePause(flags);
        setEngineState(LISTENING | PROCESSING,
                       getEngineState() & ~LISTENING & ~PROCESSING);
    }


    /**
     * Called from the <code>resume</code> method.  Override in subclasses.
     *
     * @todo Handle grammar updates
     */
    protected final boolean baseResume() throws EngineStateException {

        //Process grammars
        processGrammars();

        boolean status = handleResume();
        if (status) {
            setEngineState(0, LISTENING);
            postEngineEvent(0, LISTENING, RecognizerEvent.RECOGNIZER_LISTENING);
            setEngineState(NOT_BUFFERING, BUFFERING);
        }

        return status;
    }


    protected abstract void handleDeallocate();

    protected abstract void handlePause();

    protected abstract void handlePause(int flags);

    protected abstract boolean handleResume() throws EngineStateException;

    protected abstract boolean setGrammars(Vector grammarDefinition);

    protected abstract void handleRequestFocus();

    protected abstract void handleReleaseFocus();

    /**
     * Returns a list of engine built-in grammars.
     * @return list of buildtin grammars
     */
    public abstract Vector getBuiltInGrammars();

}
