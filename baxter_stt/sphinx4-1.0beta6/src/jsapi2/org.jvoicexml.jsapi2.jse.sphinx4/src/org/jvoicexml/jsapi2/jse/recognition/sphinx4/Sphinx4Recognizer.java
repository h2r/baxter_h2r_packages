/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/recognition/sphinx4/Sphinx4Recognizer.java $
 * Version: $LastChangedRevision: 611 $
 * Date:    $Date: 2010-12-08 15:38:52 +0100 (Mi, 08 Dez 2010) $
 * Author:  $LastChangedBy: schnelle $
 *
 * JVoiceXML - A free VoiceXML implementation.
 *
 * Copyright (C) 2005-2008 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Library General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Library General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

package org.jvoicexml.jsapi2.jse.recognition.sphinx4;

import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.Enumeration;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.sound.sampled.AudioFormat;
import javax.speech.AudioException;
import javax.speech.EngineException;
import javax.speech.EngineStateException;
import javax.speech.SpeechEventExecutor;
import javax.speech.recognition.RecognizerEvent;
import javax.speech.recognition.ResultEvent;
import javax.speech.recognition.ResultListener;
import javax.speech.recognition.RuleGrammar;

import org.jvoicexml.jsapi2.EnginePropertyChangeRequestEvent;
import org.jvoicexml.jsapi2.EnginePropertyChangeRequestListener;
import org.jvoicexml.jsapi2.jse.JseBaseAudioManager;
import org.jvoicexml.jsapi2.jse.ThreadSpeechEventExecutor;
import org.jvoicexml.jsapi2.jse.recognition.BaseResult;
import org.jvoicexml.jsapi2.jse.recognition.GrammarDefinition;
import org.jvoicexml.jsapi2.jse.recognition.JseBaseRecognizer;

import edu.cmu.sphinx.decoder.search.Token;
import edu.cmu.sphinx.frontend.DataProcessor;
import edu.cmu.sphinx.frontend.util.Microphone;
import edu.cmu.sphinx.linguist.language.grammar.Grammar;
import edu.cmu.sphinx.recognizer.Recognizer;
import edu.cmu.sphinx.recognizer.Recognizer.State;
import edu.cmu.sphinx.recognizer.StateListener;
import edu.cmu.sphinx.util.props.ConfigurationManager;
import edu.cmu.sphinx.util.props.PropertyException;
import edu.cmu.sphinx.util.props.PropertySheet;

/**
 * JSAPI wrapper for sphinx4.
 * 
 * <p>
 * Unfortunately sphinx4 provides no full support for JSAPI, so we try to build
 * our own wrapper. This is going to be a bit troublesome. Hope we can make it
 * ;-)
 * </p>
 * 
 * @author Dirk Schnelle
 * @author Stefan Radomski
 * @version $Revision: 611 $
 */
final class Sphinx4Recognizer extends JseBaseRecognizer
        implements EnginePropertyChangeRequestListener, StateListener {
    /** Logger for this class. */
    private static final Logger LOGGER = Logger
            .getLogger(Sphinx4Recognizer.class.getName());

    /**
     * Msecs to sleep before the status of the recognizer thread is checked
     * again.
     */
    private static final long SLEEP_MSEC = 50;

    /** The encapsulated recognizer. */
    private Recognizer recognizer;

    /** The input device. */
    private DataProcessor dataProcessor;

    /** The grammar manager. */
    private Grammar grammar;

    /** The result listener. */
    private final Sphinx4ResultListener resultListener;

    /**
     * The decoding thread. It points either to the single decoding thread or is
     * <code>null</code> if no recognition thread is started.
     */
    private RecognitionThread recognitionThread;

    /**
     * Construct a new object.
     */
    public Sphinx4Recognizer(SphinxRecognizerMode recognizerMode) {
        super(recognizerMode);

        String configFile = System.getProperty(
                "org.jvoicexml.jsapi2.jse.recognition.sphinx4.configPath",
                "/sphinx4.config.xml");

        URL url = Sphinx4Recognizer.class.getResource(configFile);

        // There is no config, use default config
        if (url == null) {
            LOGGER.info("Sphinx4Recognizer using default configuration.");
            url = Sphinx4Recognizer.class.getResource("default.config.xml");
        }

        try {
            final ConfigurationManager configuration = new ConfigurationManager(
                    url);

            recognizer = (Recognizer) configuration.lookup("recognizer");
            dataProcessor = (DataProcessor) configuration
                    .lookup("sphinxInputDataProcessor");
            grammar = (Grammar) configuration.lookup("srgsGrammar");

            if (!(dataProcessor instanceof SphinxInputDataProcessor)) {
                throw new EngineException("Unsupported input type");
            }

        } catch (Exception ex) {
            LOGGER.warning("error creating engine properties "
                    + ex.getMessage());
            ex.printStackTrace();
        }

        // hard-coded audio format
        ((JseBaseAudioManager) getAudioManager())
                .setEngineAudioFormat(new AudioFormat(16000, 16, 1, true, true));
        resultListener = new Sphinx4ResultListener(this);
    }

    /**
     * Called from the <code>allocate</code> method.
     * 
     * @throws EngineException
     *             if problems are encountered
     */
    public void handleAllocate() throws AudioException, EngineException,
            EngineStateException, SecurityException {

        if (recognizer == null) {
            throw new EngineException(
                    "cannot allocate: no recognizer configured!");
        }

        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("allocating recognizer...");
        }

        // Get and set input
        InputStream inputStream = ((JseBaseAudioManager) getAudioManager())
                .getInputStream();

        ((SphinxInputDataProcessor) dataProcessor).setInputStream(inputStream);

        // allocate recognizer and wait for State.READY
        recognizer.allocate();
        waitForRecognizerState(State.READY);

        // Register state and result listener
        recognizer.addResultListener(resultListener);
        recognizer.addStateListener(this);

        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("...allocated");
            LOGGER.fine("state: " + recognizer.getState());
        }

        setEngineState(CLEAR_ALL_STATE, ALLOCATED);
    }

    /**
     * Called from the <code>resume</code> method.
     */
    public boolean handleResume() {
        if (recognizer == null) {
            LOGGER.warning("no recognizer: cannot resume!");
            return false;
        }

        if (recognitionThread != null) {
            LOGGER.warning("recognition thread already started.");
            return false;
        }

        if (recognizer.getState() != State.READY) {
            LOGGER
                    .warning("Cannot resume, recognizer not ready, but in state: "
                            + recognizer.getState());
            return false;
        }

        // start data source for sphinx if it is not running
        if (dataProcessor instanceof SphinxInputDataProcessor) {
            final SphinxInputDataProcessor sidp = (SphinxInputDataProcessor) dataProcessor;
            sidp.isRunning(true);
        }

        // start the recognizer thread and wait for the recognizer to recognize
        recognitionThread = new RecognitionThread(this);
        recognitionThread.start();
        waitForRecognizerState(State.RECOGNIZING);

        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("recognition started");
        }

        return true;
    }

    /**
     * Called from the <code>pause</code> method.
     */
    public void handlePause() {
        if (recognitionThread == null) {
            throw new EngineStateException("Cannot pause, no decoder started");
        }

        // prevent further calls to recognize()
        recognitionThread.stopRecognition();

        // stop the sphinx4 frontend
        if (dataProcessor instanceof Microphone) {
            final Microphone microphone = (Microphone) dataProcessor;
            microphone.stopRecording();
        }
        if (dataProcessor instanceof SphinxInputDataProcessor) {
            final SphinxInputDataProcessor sidp = (SphinxInputDataProcessor) dataProcessor;
            sidp.isRunning(false);
        }

        // wait for the recognizer to transit from RECOGNIZING to READY
        waitForRecognizerState(State.READY);

        // get rid of the recognizer thread
        stopRecognitionThread();

    }

    /**
     * @todo Correctly implement this
     */
    public void handlePause(int flags) {
        handlePause();
    }

    /**
     * Called from the <code>deallocate</code> method.
     * 
     * According to JSAPI2 specs, pause is transitioned before dealloc.
     * 
     * @throws EngineException
     *             if this <code>Engine</code> cannot be deallocated.
     * @todo Implement this com.sun.speech.engine.BaseEngine method
     */
    public void handleDeallocate() {

        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("deallocating recognizer...");
        }

        // pause is not called before dealloc obviously
        if (recognizer.getState() != State.READY)
            handlePause();

        // Deallocate the recognizer and wait until it stops recognizing.
        recognizer.deallocate();
        waitForRecognizerState(State.DEALLOCATED);
        recognizer.resetMonitors();

        // recognizer.removeResultListener(resultListener);

        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("...deallocated");
        }

    }

    /**
     * Selector for the data processor.
     * 
     * @return The used data processor.
     */
    DataProcessor getDataProcessor() {
        return dataProcessor;
    }

    /**
     * Selector for the wrapped sphinx4 recognizer.
     * 
     * @return Recognizer
     */
    Recognizer getRecognizer() {
        return recognizer;
    }

    /**
     * Stop the recognition thread and wait until it is terminated.
     */
    private void stopRecognitionThread() {
        if (recognitionThread == null) {
            LOGGER.fine("recognition thread already stopped");
            return;
        }

        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("stopping recognition thread...");
        }
        recognitionThread.stopRecognition();

        final long maxSleepTime = 5000;
        long sleepTime = 0;

        while (recognitionThread.isAlive() && (sleepTime < maxSleepTime)) {
            try {
                Thread.sleep(SLEEP_MSEC);
                sleepTime += SLEEP_MSEC;
            } catch (InterruptedException ie) {
                if (LOGGER.isLoggable(Level.FINE)) {
                    LOGGER.fine("recognition thread interrupted");
                }
            }
        }

        recognitionThread = null;

        if (LOGGER.isLoggable(Level.FINE)) {
            LOGGER.fine("recognition thread stopped");
        }
    }

    /**
     * Get the current rule grammar or the one that produced the list of tokens
     * in case of the SRGS container.
     * 
     * @param token
     * 
     * @return Active grammar.
     */
    RuleGrammar getRuleGrammar(Token token) {
        if (grammar instanceof SRGSGrammar) {
            return ((SRGSGrammar) grammar).getRuleGrammar();
        }
        if (grammar instanceof SRGSGrammarContainer) {
            return ((SRGSGrammarContainer) grammar).getRuleGrammar(token);
        }
        return null;
    }

    /**
     * @todo: in case of grammarDefinition.size > 1, make <one-of> of all the
     *        grammars
     * @param newGrammars
     *            String[]
     * @return boolean
     */
    protected boolean setGrammars(Vector grammarDefinition) {
        if (grammar instanceof SRGSGrammar) {
            // old behavior with only a single active grammar
            if (grammarDefinition.size() == 1) {
                try {
                    ((SRGSGrammar) grammar)
                            .loadSRGS(((GrammarDefinition) grammarDefinition
                                    .get(0)).getGrammar());
                } catch (IOException ex) {
                    return false;
                }
                return true;
            } else {
                return false;
            }
        } else if (grammar instanceof SRGSGrammarContainer) {
            // the big one-of dispatcher
            try {
                ((SRGSGrammarContainer) grammar)
                        .loadGrammars(grammarDefinition);
            } catch (IOException ex) {
                System.err.println(ex);
                return false;
            }
            return true;
        }
        return false;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Vector getBuiltInGrammars() {
        return null;
    }

    public void fireResultEvent(final ResultEvent event) {
        Enumeration listeners = resultListeners.elements();
        while (listeners.hasMoreElements()) {
            ResultListener el = (ResultListener) listeners.nextElement();
            // only notify result listeners for the given grammar
            if (RuleGrammar.class.isAssignableFrom(el.getClass())
                    && event.getSource().getClass().isAssignableFrom(
                            BaseResult.class)) {
                if (((RuleGrammar) el).getReference().equals(
                        ((BaseResult) event.getSource()).getGrammar()
                                .getReference())) {
                    ((ResultListener) el).resultUpdate((ResultEvent) event);
                }
            } else {
                ((ResultListener) el).resultUpdate((ResultEvent) event);

            }
        }
    }

    public void postStartOfSpeechEvent() {
        postEngineEvent(new RecognizerEvent(this,
                RecognizerEvent.SPEECH_STARTED, 1, 1, null, null, 0));
    }

    public void postEndOfSpeechEvent() {
        // SearchGraph sg = linguist.getSearchGraph();
        // SearchGraphDumper.dumpDot("sg.dot", "foo", sg);

        postEngineEvent(new RecognizerEvent(this,
                RecognizerEvent.SPEECH_STOPPED, 1, 1, null, null, 0));
    }

    public void postProcessingEvent() {
        long states[] = setEngineState(LISTENING, PROCESSING);
        postEngineEvent(states[0], states[1],
                RecognizerEvent.RECOGNIZER_PROCESSING, (long) 0);
    }

    public void postListeningEvent() {
        long states[] = setEngineState(PROCESSING, LISTENING);
        postEngineEvent(states[0], states[1],
                RecognizerEvent.RECOGNIZER_LISTENING, (long) 0);

    }

    /**
     * {@inheritDoc}
     */
    public void postResultEvent(final ResultEvent resultEvent) {
        super.postResultEvent(resultEvent);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected EnginePropertyChangeRequestListener getChangeRequestListener() {
        return this;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void propertyChangeRequest(EnginePropertyChangeRequestEvent event) {
        // TODO Auto-generated method stub

    }

    @Override
    protected void handleRequestFocus() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void handleReleaseFocus() {
        // TODO Auto-generated method stub

    }

    /**
     * This method gets called by the Sphinx4 Recognizer, when its status
     * changes. We just notify all threads waiting in waitForRecognizerState on
     * this object.
     * 
     * @param status
     */
    @Override
    public synchronized void statusChanged(State status) {
        notifyAll();
    }

    /**
     * Wait for the recognizer to enter the given state.
     * 
     * @param status
     *            The state of the recognizer to wait for.
     */
    private synchronized void waitForRecognizerState(State status) {
        while (recognizer.getState() != status) {
            try {
                wait();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        LOGGER.info("Sphinx4Recognizer in state: " + status);
    }

    @Override
    public void newProperties(PropertySheet ps) throws PropertyException {
        // TODO Auto-generated method stub

    }
}
