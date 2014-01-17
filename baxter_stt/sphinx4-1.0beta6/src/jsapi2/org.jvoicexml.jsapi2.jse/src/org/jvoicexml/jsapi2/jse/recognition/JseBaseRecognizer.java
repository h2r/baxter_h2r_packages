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
package org.jvoicexml.jsapi2.jse.recognition;

import javax.sound.sampled.AudioFormat;
import javax.speech.AudioManager;
import javax.speech.SpeechEventExecutor;
import javax.speech.recognition.Grammar;
import javax.speech.recognition.GrammarException;
import javax.speech.recognition.GrammarManager;
import javax.speech.recognition.RecognizerMode;
import javax.speech.recognition.Result;
import javax.speech.recognition.ResultEvent;

import org.jvoicexml.jsapi2.jse.ThreadSpeechEventExecutor;
import org.jvoicexml.jsapi2.recognition.BaseRecognizer;


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
public abstract class JseBaseRecognizer extends BaseRecognizer {
    /**
     * Create a new Recognizer in the DEALLOCATED state.
     */
    public JseBaseRecognizer() {
    }

    /**
     * Create a new Recognizer in the DEALLOCATED state.
     * @param mode the recognizer mode
     */
    public JseBaseRecognizer(final RecognizerMode mode) {
        super(mode);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected GrammarManager createGrammarManager() {
        return new BaseGrammarManager(this);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected AudioManager createAudioManager() {
        final AudioFormat format = new AudioFormat(8000f, 16, 1, true, true);
        return new BaseRecognizerAudioManager(format);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected boolean processGrammar(final Grammar grammar)
        throws GrammarException {
        if (grammar instanceof BaseRuleGrammar) {
            BaseRuleGrammar baseRuleGrammar = (BaseRuleGrammar) grammar;
            return baseRuleGrammar.commitChanges();
        }
        return false;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected void postResultEvent(final Result result, final ResultEvent event,
            final SpeechEventExecutor executor) {
        final BaseResult base = (BaseResult) result;
        base.postResultEvent(executor, event);
    }


    /**
     * {@inheritDoc}
     */
    @Override
    protected SpeechEventExecutor createSpeechEventExecutor() {
        return new ThreadSpeechEventExecutor();
    }
}
