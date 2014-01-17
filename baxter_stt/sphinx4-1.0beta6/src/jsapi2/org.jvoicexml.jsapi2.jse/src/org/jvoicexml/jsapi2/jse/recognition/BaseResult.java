/**
 * Copyright 1998-2003 Sun Microsystems, Inc.
 *
 * See the file "license.terms" for information on usage and
 * redistribution of this file, and for a DISCLAIMER OF ALL
 * WARRANTIES.
 */

/**
 * Very simple implementation of JSAPI Result, FinalResult,
 * FinalRuleResult, and FinalDictationResult.
 *
 * Ignores many things like N-Best, partial-results, etc.
 *
 * @version 1.9 09/09/99 14:24:41
 */
package org.jvoicexml.jsapi2.jse.recognition;

import java.io.Serializable;
import java.util.Enumeration;
import java.util.StringTokenizer;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.speech.AudioSegment;
import javax.speech.SpeechEventExecutor;
import javax.speech.recognition.FinalResult;
import javax.speech.recognition.FinalRuleResult;
import javax.speech.recognition.Grammar;
import javax.speech.recognition.GrammarException;
import javax.speech.recognition.RecognizerProperties;
import javax.speech.recognition.Result;
import javax.speech.recognition.ResultEvent;
import javax.speech.recognition.ResultListener;
import javax.speech.recognition.ResultStateException;
import javax.speech.recognition.ResultToken;
import javax.speech.recognition.RuleAlternatives;
import javax.speech.recognition.RuleComponent;
import javax.speech.recognition.RuleCount;
import javax.speech.recognition.RuleGrammar;
import javax.speech.recognition.RuleParse;
import javax.speech.recognition.RuleReference;
import javax.speech.recognition.RuleSequence;
import javax.speech.recognition.RuleTag;
import javax.speech.recognition.RuleToken;

import org.jvoicexml.jsapi2.recognition.BaseResultToken;

public class BaseResult implements Result, FinalResult, FinalRuleResult, Serializable, Cloneable {
    /** Logger for this class. */
    private static final Logger LOGGER =
            Logger.getLogger(BaseResult.class.getName());

    private Vector resultListeners;
    private ResultToken tokens[];
    private int nTokens;
    private transient Grammar grammar;
    private int state;
    private int confidenceLevel;

    protected String[] tags;
    private String   ruleName;

    /**
     * Create an empty result.
     * @exception GrammarException error evaluating the grammar
     */
    public BaseResult() throws GrammarException {
        this(null);
    }

    /**
     * Create an empty result.
     * @param gram the grammar
     * @exception GrammarException error evaluating the grammar
     */
    public BaseResult(final Grammar gram) throws GrammarException {
        this(gram, null);
    }

    /**
     * Create a result with a result string
     * @param gram the grammar
     * @param result the result string
     * @exception GrammarException error evaluating the grammar
     */
    public BaseResult(Grammar gram, String result) throws GrammarException {
        resultListeners = new Vector();
        grammar = gram;
        confidenceLevel = RecognizerProperties.UNKNOWN_CONFIDENCE;
        if (result == null) {
            state = Result.UNFINALIZED;
        } else {
            final boolean success = tryTokens(gram, result);
            if (success) {
                state = Result.ACCEPTED;
            } else {
                state = Result.REJECTED;
            }
        }
    }

    /**
     * Copy a result. If the result to be copied is a BaseResult
     * then clone it otherwise create a BaseResult and copy the
     * tokens onto it.
     * @param result the result to copy to
     * @return copied result
     * @exception GrammarException if the related grammar could not be evaluated
     */
    static BaseResult copyResult(Result result) throws GrammarException {
        BaseResult copy = null;
        if (result instanceof BaseResult) {
            try {
                copy = (BaseResult) ((BaseResult)result).clone();
            } catch (CloneNotSupportedException e) {
                LOGGER.warning("ERROR: " + e);
            }
            return copy;
        } else {
            copy = new BaseResult(result.getGrammar());
            copy.nTokens = result.getNumTokens();
            copy.tokens = new ResultToken[copy.nTokens];
            for (int i = 0; i < result.getNumTokens(); i++) {
                ResultToken sourceToken = result.getBestToken(i);
                BaseResultToken destinationToken = new BaseResultToken(copy, sourceToken.getText());

                destinationToken.setConfidenceLevel(sourceToken.getConfidenceLevel());
                destinationToken.setStartTime(sourceToken.getStartTime());
                destinationToken.setEndTime(sourceToken.getEndTime());

                copy.tokens[i] = destinationToken;
            }
            return copy;
        }
    }

//////////////////////
// Begin Result Methods
//////////////////////
    /**
     * Return the current state of the Result object.
     * From javax.speech.recognition.Result.
     */
    public int getResultState() {
        return state;
    }

    /**
     * Return the grammar that goes with this Result.
     * From javax.speech.recognition.Result.
     */
    public Grammar getGrammar() {
        return grammar;
    }

    /**
     * Return the number of finalized tokens in the Result.
     * From javax.speech.recognition.Result.
     */
    public int getNumTokens() {
        return nTokens;
    }

    /**
     * Return the best guess for the nth token.
     * From javax.speech.recognition.Result.
     */
    public ResultToken getBestToken(int nth) throws IllegalArgumentException {
        if ((nth < 0) || (nth > (nTokens-1))) {
            throw new IllegalArgumentException("Token index out of range.");
        }
        return tokens[nth];
    }

    /**
     * Return the best guess tokens for the Result.
     * From javax.speech.recognition.Result.
     */
    public ResultToken[] getBestTokens() {
        return tokens;
    }

    /**
     * Return the current guess of the tokens following the unfinalized
     * tokens.
     * From javax.speech.recognition.Result.
     */
    public ResultToken[] getUnfinalizedTokens() {
        if (getResultState() == Result.ACCEPTED ||
            getResultState() == Result.REJECTED)
            return new ResultToken[0];

        int numUnfinalizedTokens = getBestTokens().length - getNumTokens();

        ResultToken[] unfinalizedTokens = new ResultToken[numUnfinalizedTokens];

        for (int i = 0; i < numUnfinalizedTokens; ++i)
            unfinalizedTokens[i] = getBestTokens()[i + getNumTokens()];

        return unfinalizedTokens;
    }

    /**
     * Add a ResultListener to this Result.
     * From javax.speech.recognition.Result.
     */
    public void addResultListener(ResultListener listener) {
        if (!resultListeners.contains(listener)) {
            resultListeners.addElement(listener);
        }
    }

    /**
     * Remove a ResultListener from this Result.
     * From javax.speech.recognition.Result.
     */
    public void removeResultListener(ResultListener listener) {
        resultListeners.removeElement(listener);
    }
//////////////////////
// End Result Methods
//////////////////////

//////////////////////
// Begin FinalResult Methods
//////////////////////
    /**
     * Returns true if the Recognizer has training information available
     * for this result.
     * From javax.speech.recognition.FinalResult.
     */
    public boolean isTrainingInfoAvailable() throws ResultStateException {
        checkResultState(UNFINALIZED);
        return false;
    }

    /**
     * Release training info for this FinalResult.
     * From javax.speech.recognition.FinalResult.
     */
    public void releaseTrainingInfo() throws ResultStateException {
        checkResultState(UNFINALIZED);
    }

    /**
     * Inform the recognizer of a correction to one or more tokens in
     * a FinalResult to the recognizer can re-train itself.
     * From javax.speech.recognition.FinalResult.
     */
    public void tokenCorrection(String correctTokens[],
                                ResultToken fromToken,
                                ResultToken toToken,
                                int correctionType)
        throws ResultStateException, IllegalArgumentException
    {
        checkResultState(UNFINALIZED);
    }

    /**
     * Determine if audio is available for this FinalResult.
     * From javax.speech.recognition.FinalResult.
     */
    public boolean isAudioAvailable() throws ResultStateException {
        checkResultState(UNFINALIZED);
        return false;
    }

    /**
     * Release the audio for this FinalResult.
     * From javax.speech.recognition.FinalResult.
     */
    public void releaseAudio() throws ResultStateException {
        checkResultState(UNFINALIZED);
    }

    /**
     * Get the audio for this FinalResult.
     * From javax.speech.recognition.FinalResult.
     */
    public AudioSegment getAudio() throws ResultStateException {
        checkResultState(UNFINALIZED);
        return null;
    }

    /**
     * Get the audio for this FinalResult.
     * From javax.speech.recognition.FinalResult.
     */
    public AudioSegment getAudio(ResultToken from, ResultToken to)
        throws ResultStateException
    {
        checkResultState(UNFINALIZED);
        return null;
    }
//////////////////////
// End FinalResult Methods
//////////////////////

//////////////////////
// Begin FinalRuleResult Methods
//////////////////////
    /**
     * Return the number of guesses for this FinalRuleResult.
     * From javax.speech.recognition.FinalRuleResult.
     */
    public int getNumberAlternatives() throws ResultStateException {
        checkResultState(UNFINALIZED);
        return 1;
    }

    /**
     * Get the nBest token sequence for this FinalRuleResult.
     * From javax.speech.recognition.FinalRuleResult.
     */
    public ResultToken[] getAlternativeTokens(int nBest)
        throws ResultStateException
    {
        checkResultState(UNFINALIZED);
        if (!(grammar instanceof RuleGrammar)) {
            throw new ResultStateException("Result is not a FinalRuleResult");
        }
        if (nBest == 0) {
            return getBestTokens();
        }
        //[[[WDW - throw InvalidArgumentException?]]]
        return null;
    }

    /**
     * Get the RuleGrammar matched by the nBest guess for this FinalRuleResult.
     * From javax.speech.recognition.FinalRuleResult.
     */
    public Grammar getGrammar(int nBest)
        throws ResultStateException
    {
        checkResultState(UNFINALIZED);
        if (!(grammar instanceof RuleGrammar)) {
            throw new ResultStateException("Result is not a FinalRuleResult");
        }
        if (nBest == 0) {
            return (RuleGrammar) grammar;
        }
        //[[[WDW - throw InvalidArgumentException?]]]
        return null;
    }

    /**
     * Get the RuleGrammar matched by the nBest guess for this FinalRuleResult.
     * From javax.speech.recognition.FinalRuleResult.
     */
    public String getRuleName(int nBest)
        throws ResultStateException
    {
        checkResultState(UNFINALIZED);
        if (!(grammar instanceof RuleGrammar)) {
            throw new ResultStateException("Result is not a FinalRuleResult");
        }
        if (nBest == 0) {
            return ruleName;
        }
        //[[[WDW - throw InvalidArgumentException?]]]
        return null;
    }

    /**
     * Return the list of tags matched by the best-guess token sequence.
     * From javax.speech.recognition.FinalRuleResult.
     */
    public Object[] getTags(int nBest)
            throws IllegalArgumentException, ResultStateException
    {
        checkResultState(UNFINALIZED);
        if (!(grammar instanceof RuleGrammar)) {
            throw new ResultStateException("Result is not a FinalRuleResult");
        }
        return tags;
    }

    public RuleReference getRuleReference(int nBest) throws ResultStateException, IllegalArgumentException, IllegalStateException {
        throw new RuntimeException("BaseResult.getRuleReference NOT IMPLEMENTED");
        //return null;
    }


//////////////////////
// End FinalRuleResult Methods
//////////////////////

//////////////////////
// Begin FinalDictationResult Methods
//////////////////////
    /**
     * NOT IMPLEMENTED YET.
     * Get a set of alternative token guesses for a single known token or
     * sequence of tokens.
     */
/*    public ResultToken[][] getAlternativeTokens(ResultToken from,
                                                ResultToken to,
                                                int max)
        throws ResultException, IllegalArgumentException
    {
        checkResultState(UNFINALIZED);
        if (!(grammar instanceof DictationGrammar)) {
            throw new ResultException("Result is not a FinalDicationResult");
        }
        return null;
    }*/
//////////////////////
// End FinalDictationResult Methods
//////////////////////

//////////////////////
// Begin utility methods for sending ResultEvents
//////////////////////
    /**
     * Utility function to generate result event and post it to the event queue.
     * Eventually fireAudioReleased will be called
     * by dispatchSpeechEvent as a result of this action.
     * @param speechEventExecutor SpeechEventExecutor
     * @param event ResultEvent
     */
    public void postResultEvent(final SpeechEventExecutor speechEventExecutor,
            final ResultEvent event) {
          try {
              speechEventExecutor.execute(new Runnable() {
                  public void run() {
                      if (LOGGER.isLoggable(Level.FINE)) {
                          LOGGER.fine("notifying event " + event);
                      }
                      fireResultEvent(event);
                  }
              });
          } catch (RuntimeException ex) {
              LOGGER.warning(ex.getLocalizedMessage());
          }
      }

      /**
       * Utility function to send a result event to all result
       * listeners.
       */
      public void fireResultEvent(final ResultEvent event) {
          if (resultListeners != null) {
              final Enumeration enumeration = resultListeners.elements();
              while (enumeration.hasMoreElements()) {
                  ResultListener rl =
                      (ResultListener) enumeration.nextElement();
                  rl.resultUpdate(event);
              }
          }
      }

    /**
     * Utility function to generate AUDIO_RELEASED event and post it
     * to the event queue.  Eventually fireAudioReleased will be called
     * by dispatchSpeechEvent as a result of this action.
     */
    public void postAudioReleased() {
       /* SpeechEventUtilities.postSpeechEvent(
            this,
            new ResultEvent(this, ResultEvent.AUDIO_RELEASED));*/
    }

    /**
     * Utility function to send a AUDIO_RELEASED event to all result
     * listeners.
     */
    public void fireAudioReleased(ResultEvent event) {
    /*    Enumeration E;
	if (resultListeners != null) {
            E = resultListeners.elements();
            while (E.hasMoreElements()) {
                ResultListener rl = (ResultListener) E.nextElement();
                rl.audioReleased(event);
            }
        }*/
    }

    /**
     * Utility function to generate GRAMMAR_FINALIZED event and post it
     * to the event queue.  Eventually fireGrammarFinalized will be called
     * by dispatchSpeechEvent as a result of this action.
     */
    public void postGrammarFinalized() {
       /* SpeechEventUtilities.postSpeechEvent(
            this,
            new ResultEvent(this, ResultEvent.GRAMMAR_FINALIZED));*/
    }

    /**
     * Utility function to send a GRAMMAR_FINALIZED event to all result
     * listeners.
     */
    public void fireGrammarFinalized(ResultEvent event) {
        /*Enumeration E;
	if (resultListeners != null) {
            E = resultListeners.elements();
            while (E.hasMoreElements()) {
                ResultListener rl = (ResultListener) E.nextElement();
                rl.grammarFinalized(event);
            }
        }*/
    }

    /**
     * Utility function to generate RESULT_ACCEPTED event and post it
     * to the event queue.  Eventually fireResultAccepted will be called
     * by dispatchSpeechEvent as a result of this action.
     */
    public void postResultAccepted() {
        /*setResultState(Result.ACCEPTED);
        SpeechEventUtilities.postSpeechEvent(
            this,
            new ResultEvent(this, ResultEvent.RESULT_ACCEPTED));*/
    }

    /**
     * Utility function to send a RESULT_ACCEPTED event to all result
     * listeners.
     */
    public void fireResultAccepted(ResultEvent event) {
        /*Enumeration E;
	if (resultListeners != null) {
            E = resultListeners.elements();
            while (E.hasMoreElements()) {
                ResultListener rl = (ResultListener) E.nextElement();
                rl.resultAccepted(event);
            }
        }*/
    }

    /**
     * Utility function to generate RESULT_CREATED event and post it
     * to the event queue.  Eventually fireResultCreated will be called
     * by dispatchSpeechEvent as a result of this action.
     */
    public void postResultCreated() {
        /*SpeechEventUtilities.postSpeechEvent(
            this,
            new ResultEvent(this, ResultEvent.RESULT_CREATED));*/
    }

    /**
     * Utility function to send a RESULT_CREATED event to all result
     * listeners.
     */
    public void fireResultCreated(ResultEvent event) {
      /*  Enumeration E;
	if (resultListeners != null) {
            E = resultListeners.elements();
            while (E.hasMoreElements()) {
                ResultListener rl = (ResultListener) E.nextElement();
                rl.resultCreated(event);
            }
        }*/
    }

    /**
     * Utility function to generate RESULT_REJECTED event and post it
     * to the event queue.  Eventually fireResultRejected will be called
     * by dispatchSpeechEvent as a result of this action.
     */
    public void postResultRejected() {
       /* setResultState(Result.REJECTED);
        SpeechEventUtilities.postSpeechEvent(
            this,
            new ResultEvent(this, ResultEvent.RESULT_REJECTED));*/
    }

    /**
     * Utility function to send a RESULT_REJECTED event to all result
     * listeners.
     */
    public void fireResultRejected(ResultEvent event) {
     /*   Enumeration E;
	if (resultListeners != null) {
            E = resultListeners.elements();
            while (E.hasMoreElements()) {
                ResultListener rl = (ResultListener) E.nextElement();
                rl.resultRejected(event);
            }
        }*/
    }

    /**
     * Utility function to generate RESULT_UPDATED event and post it
     * to the event queue.  Eventually fireResultUpdated will be called
     * by dispatchSpeechEvent as a result of this action.
     */
    public void postResultUpdated() {
       /* SpeechEventUtilities.postSpeechEvent(
            this,
            new ResultEvent(this, ResultEvent.RESULT_UPDATED));*/
    }

    /**
     * Utility function to send a RESULT_UPDATED event to all result
     * listeners.
     */
    public void fireResultUpdated(ResultEvent event) {
      /*  Enumeration E;
	if (resultListeners != null) {
            E = resultListeners.elements();
            while (E.hasMoreElements()) {
                ResultListener rl = (ResultListener) E.nextElement();
                rl.resultUpdated(event);
            }
        }*/
    }

    /**
     * Utility function to generate TRAINING_INFO_RELEASED event and post it
     * to the event queue.  Eventually fireTrainingInfoReleased will be called
     * by dispatchSpeechEvent as a result of this action.
     */
    public void postTrainingInfoReleased() {
     /*   SpeechEventUtilities.postSpeechEvent(
            this,
            new ResultEvent(this, ResultEvent.TRAINING_INFO_RELEASED));*/
    }

    /**
     * Utility function to send a TRAINING_INFO_RELEASED event to all result
     * listeners.
     */
    public void fireTrainingInfoReleased(ResultEvent event) {
     /*   Enumeration E;
	if (resultListeners != null) {
            E = resultListeners.elements();
            while (E.hasMoreElements()) {
                ResultListener rl = (ResultListener) E.nextElement();
                rl.trainingInfoReleased(event);
            }
        }*/
    }
//////////////////////
// End utility methods for sending ResultEvents
//////////////////////

    /**
     * Utility function to set the number of finalized tokens in the Result.
     * @param n int
     */
    public void setNumTokens(int n){
        nTokens = n;
    }

    /**
     * Simple implementation of tags.
     *
     * ATTENTION: This method changes the ResultToken content.
     * <p>
     * This implementation replaces each token text by tag information,
     * if it exists.
     * </p>
     *
     * @todo the tag information can not be only text! It can be anything.
     *
     * @param rt the result tokens
     * @param rc the rule component that will be analyzed
     * @param iPos the position in rt of next token that will be appear
     * in the graph
     * @return int
     */
    private int applyTags(ResultToken rt[], final RuleComponent rc,
                          int iPos) {
        if (rc instanceof RuleReference) {
            return iPos;
        } else if (rc instanceof RuleToken) {
            return iPos + 1;
        } else if (rc instanceof RuleAlternatives) {
            return applyTags(rt, ((RuleAlternatives) rc).getRuleComponents()[0],
                             iPos);
        } else if (rc instanceof RuleSequence) {
            for (RuleComponent r : ((RuleSequence) rc).getRuleComponents()) {
                iPos = applyTags(rt, r, iPos);
            }
            return iPos;
        } else if (rc instanceof RuleTag) {
            String tag = (String) ((RuleTag) rc).getTag();

            //assumes that ruleTag component appears after RuleToken component
            rt[iPos - 1] = new BaseResultToken(rt[iPos - 1].getResult(), tag);
            return iPos;
        } else if (rc instanceof RuleCount) {
            return applyTags(rt, ((RuleCount) rc).getRuleComponent(), iPos);
        }

        return iPos;
    }

    /**
     * Utility function to set the resultTokens.
     * @param rt
     */
    public void setTokens(ResultToken rt[]) {
        setTokens(rt, false);
    }

    /**
     * Utility function to set the resultTokens.
     * @param rt
     * @param replaceTags if true, tokens must be replaced by tags content.
     */
    public void setTokens(ResultToken rt[], boolean replaceTags) {
        tokens = new ResultToken[rt.length];
        int i = 0;
        for (ResultToken resultToken : rt) {
            tokens[i++] = resultToken;
        }

        if (replaceTags) {
            final RuleParse ruleParse = parse(0);
            if (ruleParse != null) {
                applyTags(tokens, ruleParse.getParse(), 0);
            }
        }
    }

    /**
     * Concatenate the best tokens in the Result.
     */
    public String toString() {
        if (nTokens==0)  /** @todo change this. Is possible a result has 0 tokens?? */
            return "";

        StringBuffer sb = new StringBuffer(getBestToken(0).getText());
        for (int i = 1; i < getNumTokens(); i++)
            sb.append(" " + getBestToken(i).getText());
        return sb.toString();
    }


    /**
     * Utility function to set the result state.
     */
    public void setResultState(int state) {
        this.state = state;
    }

    /**
     * If the result is in the given state, throw a ResultStateError.
     */
    protected void checkResultState(int state) throws ResultStateException {
        if (getResultState() == state) {
            throw new ResultStateException("Invalid ResultState: " +
                                       getResultState());
        }
    }

    /**
     * Set the grammar that goes with this Result.  NOT JSAPI.
     * @param gram the grammar
     */
    public void setGrammar(Grammar gram) {
        grammar = gram;
    }

    /**
     * Try to set the Grammar and tokens of this result.
     * @param gram the related grammar
     * @param result the retrieved recognition result
     * @return <code>true</code> if the grammar matches the tokens.
     */
    public boolean tryTokens(final Grammar gram, final String result)
        throws GrammarException {
        if ((result == null) || (gram == null)) {
            return false;
        }

        if (gram instanceof RuleGrammar) {
            try {
                final RuleGrammar rule = (RuleGrammar) gram;
                final RuleParse rp = rule.parse(result, null);
                if (rp != null) {
                    grammar = gram;
                    tags = (String[])rp.getTags();   /** @todo Danger. RRR */
                    ruleName = rp.getRuleReference().getRuleName();
                    final StringTokenizer st = new StringTokenizer(result);
                    nTokens = st.countTokens();
                    int i = 0;
                    tokens = new ResultToken[nTokens];
                    while (st.hasMoreTokens()) {
                        /** @todo information about startTime, endTime and confidenceLevel */
                        tokens[i] = new BaseResultToken(this, st.nextToken()); 
                        ++i;
                    }
                    return true;
                }
            } catch (GrammarException e) {
                return false;
            }
        }
        return false;
    }

/** @todo Implement the following methods */



    public RuleParse parse(int nBest) throws IllegalArgumentException,
            ResultStateException {
         final ResultToken[] rt = getAlternativeTokens(0);
         final String tokens[] = new String[rt.length];
         for (int i = 0; i < rt.length; ++i) {
             tokens[i] = rt[i].getText();
         }

         try {
             return ((RuleGrammar) getGrammar()).parse(tokens, ((BaseRuleGrammar)getGrammar()).getRoot());
         } catch (GrammarException e) {
             e.printStackTrace();
         }
        return null;
    }

    public void setConfidenceLevel(int confidenceLevel) {
        if (confidenceLevel < RecognizerProperties.MIN_CONFIDENCE ||
            confidenceLevel > RecognizerProperties.MAX_CONFIDENCE)
            throw new IllegalArgumentException("Invalid confidenceThreshold: " +
                                               confidenceLevel);
        this.confidenceLevel = confidenceLevel;
    }


    public int getConfidenceLevel() throws ResultStateException {
        return getConfidenceLevel(0);
    }

    public int getConfidenceLevel(int nBest) throws IllegalArgumentException,
            ResultStateException {
        //uncommented - see JSAPI2/FinalResult#getConfidenceLevel
        // quote: "For a REJECTED result, a useful confidence level 
        //              MAY be returned, but this is application 
        //              and platform dependent."
        if (state != FinalResult.ACCEPTED 
                && state != FinalResult.REJECTED 
                && state != FinalResult.DONT_KNOW 
                && state != FinalResult.MISRECOGNITION 
                && state != FinalResult.USER_CHANGE)
            //above code could simply check for FinalResult.UNFINALIZED 
            //but the implementation above is more error prone 
            //(e.g. if the ResultState was not correctly set to UNFINALIZED)
           throw new ResultStateException();

        if (nBest<0 || nBest>=getNumberAlternatives())
           throw new IllegalArgumentException("nBest out of valid range!");

       return confidenceLevel;
    }

    /**
     *
     * @todo TEMPORARY! TRASH IT ASAP!
     *
     * @param result String
     */
    public void setResult(final String result) throws Exception {
        throw new Exception("Method temporally commented. Try use setTokens()!!");
        /*StringTokenizer st = new StringTokenizer(result);
        nTokens = st.countTokens();
        int i = 0;
        theText = new String[nTokens];
        while (st.hasMoreTokens()) {
            theText[i++]=st.nextToken();
        }*/
    }



}
