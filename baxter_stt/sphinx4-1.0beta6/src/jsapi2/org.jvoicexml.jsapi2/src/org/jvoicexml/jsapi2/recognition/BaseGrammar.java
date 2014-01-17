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

package org.jvoicexml.jsapi2.recognition;

import java.util.Enumeration;
import java.util.Vector;

import javax.speech.SpeechEventExecutor;
import javax.speech.SpeechLocale;
import javax.speech.recognition.Grammar;
import javax.speech.recognition.GrammarEvent;
import javax.speech.recognition.GrammarListener;
import javax.speech.recognition.GrammarManager;
import javax.speech.recognition.Recognizer;
import javax.speech.recognition.RecognizerMode;
import javax.speech.recognition.ResultEvent;
import javax.speech.recognition.ResultListener;


/**
 * Implementation of {@link javax.speech.recognition.Grammar}.
 *
 * <p>
 * This implementation adds itself as a {@link ResultListener} to the current
 * recognizer and forwards the received events. Currently there is no filtering
 * if the result matches this grammar.
 * </p>
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version $Revision: 1370 $
 */
public class BaseGrammar implements Grammar, ResultListener {
    /** Reference to the recognizer. */
    private transient Recognizer recognizer;

    /** The language. */
    private SpeechLocale locale;

    /** Registered listeners for grammar changes. */
    private transient Vector grammarListeners;

    /** Registered listeners for result events. */
    private transient Vector resultListeners;

    /** Name of this grammar. */
    protected String name;

    /**
     * <code>true</code> if this grammar is active.
     * only changed by commit and rec focus.
     */
    protected boolean grammarActive;

    /** The activation mode of this grammar. */
    private int activationMode;

    /** <code>true</code> if this grammar can be activated. */
    private boolean activatable;

    /**
     * Creates a new BaseGrammar.
     * @param rec the BaseRecognizer for this Grammar.
     * @param grammarName the name of this Grammar.
     */
    public BaseGrammar(final Recognizer rec, final String grammarName) {
        grammarListeners = new Vector();
        resultListeners = new Vector();
        recognizer = rec;
        if (rec != null) {
            recognizer.addResultListener(this);
        }
        name = grammarName;
        grammarActive = false;
        activatable = false;
        activationMode = ACTIVATION_FOCUS;
    }

    /**
     * {@inheritDoc}
     */
    public final Recognizer getRecognizer() {
        return recognizer;
    }

    /**
     * {@inheritDoc}
     */
    public final String getReference() {
        return name;
    }

    /**
     * {@inheritDoc}
     */
    public final void setActivatable(final boolean value) {
        activatable = value;
    }

    /**
     * {@inheritDoc}
     */
    public final boolean isActivatable() {
        return activatable;
    }

    /**
     * {@inheritDoc}
     */
    public void setActivationMode(final int mode)
        throws IllegalArgumentException {
        if ((mode != ACTIVATION_GLOBAL)
            && (mode != ACTIVATION_MODAL)
            && (mode != ACTIVATION_FOCUS)
            && (mode != ACTIVATION_INDIRECT)) {
            throw new IllegalArgumentException("Invalid ActivationMode");
        } else if (mode != activationMode) {
            activationMode = mode;
        }
    }

    /**
     * {@inheritDoc}
     */
    public int getActivationMode() {
        return activationMode;
    }

    /**
     * {@inheritDoc}
     */
    public boolean isActive() {
        if (!isActivatable()) {
            return false;
        } else if (getActivationMode() == Grammar.ACTIVATION_GLOBAL) {
            return true;
        } else if (recognizer.testEngineState(Recognizer.FOCUSED)) {
            if (getActivationMode() == Grammar.ACTIVATION_MODAL) {
                return true;
//            } else if (!hasModalGrammars) {
//                return true;
            }
        }
        return false;
    }

    /**
     * {@inheritDoc}
     */
    public final GrammarManager getGrammarManager() {
        return recognizer.getGrammarManager();
    }

    /**
     * {@inheritDoc}
     */
    public final void addGrammarListener(final GrammarListener listener) {
        if (!grammarListeners.contains(listener)) {
            grammarListeners.addElement(listener);
        }
    }

    /**
     * {@inheritDoc}
     */
    public final void removeGrammarListener(final GrammarListener listener) {
        grammarListeners.removeElement(listener);
    }

    /**
     * {@inheritDoc}
     */
    public final void addResultListener(final ResultListener listener) {
        if (!resultListeners.contains(listener)) {
            resultListeners.addElement(listener);
        }
    }

    /**
     * {@inheritDoc}
     */
    public final void removeResultListener(final ResultListener listener) {
        resultListeners.removeElement(listener);
    }

    /**
     * Retrieves the locale of this grammar.
     * @return the locale
     */
    protected final SpeechLocale getSpeechLocale() {
        if (locale == null) {
            final RecognizerMode mode =
                (RecognizerMode) recognizer.getEngineMode();
            final SpeechLocale[] locales = mode.getSpeechLocales();
            if (locales != null) {
                locale = locales[0];
            }
            if (locale == null) {
                locale = SpeechLocale.getDefault();
            }
        }
        return locale;
    }

    /**
     * Sets the speech locale of this grammar.
     * @param speechLocale the locale
     */
    protected final void setSpeechLocale(final SpeechLocale speechLocale) {
        locale = speechLocale;
    }

    /**
     * Utility function to generate grammar event and post it to the event
     * queue.
     * @param event the event to post
     *
     */
    private void postGrammarEvent(final GrammarEvent event) {
        final SpeechEventExecutor executor =
            recognizer.getSpeechEventExecutor();
        try {
            executor.execute(new Runnable() {
                public void run() {
                    fireGrammarEvent(event);
                }
            });
        } catch (RuntimeException ex) {
            ex.printStackTrace();
        }
    }

    /**
     * Utility function to send a grammar event to all grammar
     * listeners.
     * <p>
     * This method runs within the configures {@link SpeechEventExecutor}.
     * </p>
     * @param event the event to fire
     */
    private void fireGrammarEvent(final GrammarEvent event) {
        Enumeration enumeration;
        if (resultListeners != null) {
            enumeration = resultListeners.elements();
            while (enumeration.hasMoreElements()) {
                GrammarListener listener =
                    (GrammarListener) enumeration.nextElement();
                listener.grammarUpdate(event);
            }
        }
    }

    /**
     * Utility function to send a GRAMMAR_ACTIVATED event to all result
     * listeners.
     */
    public void postGrammarActivated() {
        final GrammarEvent event =
            new GrammarEvent(this, GrammarEvent.GRAMMAR_ACTIVATED,
                    true, false, null);
        postGrammarEvent(event);
    }

    /**
     * Utility function to send a GRAMMAR_CHANGES_COMMITTED event to all result
     * listeners.
     */
    public void postGrammarChangesCommitted() {
        final GrammarEvent event =
            new GrammarEvent(this, GrammarEvent.GRAMMAR_CHANGES_COMMITTED,
                    false, true, null);
        postGrammarEvent(event);
    }

    /**
     * Utility function to send a GRAMMAR_CHANGES_REJECTED event to all result
     * listeners.
     */
    public void postGrammarChangesRejected() {
        final GrammarEvent event =
            new GrammarEvent(this, GrammarEvent.GRAMMAR_CHANGES_REJECTED,
                    false, true, null);
        postGrammarEvent(event);
    }

    /**
     * Utility function to send a GRAMMAR_DEACTIVATED event to all result
     * listeners.
     */
    public void postGrammarDeactivated() {
        final GrammarEvent event =
            new GrammarEvent(this, GrammarEvent.GRAMMAR_DEACTIVATED,
                    true, false, null);
        postGrammarEvent(event);
    }

    /**
     * {@inheritDoc}
     */
    public void resultUpdate(final ResultEvent event) {
        final int id = event.getId();
        // TODO correct the event filter.
        if ((id != ResultEvent.RESULT_ACCEPTED)
                && (id != ResultEvent.RESULT_REJECTED)) {
            return;
        }

        // We are running in the speech event executor, so there is no need to
        // create a new one.
        Enumeration enumeration = resultListeners.elements();
        while (enumeration.hasMoreElements()) {
            final ResultListener listener =
                (ResultListener) enumeration.nextElement();
            listener.resultUpdate(event);
        }
    }
}

