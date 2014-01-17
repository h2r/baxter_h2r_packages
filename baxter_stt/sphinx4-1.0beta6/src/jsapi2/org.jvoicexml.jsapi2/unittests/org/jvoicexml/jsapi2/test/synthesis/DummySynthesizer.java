package org.jvoicexml.jsapi2.test.synthesis;
import javax.speech.AudioException;
import javax.speech.AudioManager;
import javax.speech.EngineException;
import javax.speech.EngineStateException;
import javax.speech.SpeechEventExecutor;
import javax.speech.VocabularyManager;
import javax.speech.synthesis.Speakable;

import org.jvoicexml.jsapi2.DummyAudioManager;
import org.jvoicexml.jsapi2.DummySpeechEventExecutor;
import org.jvoicexml.jsapi2.EnginePropertyChangeRequestListener;
import org.jvoicexml.jsapi2.synthesis.BaseSynthesizer;

/**
 * 
 */

/**
 * @author Dirk Schnelle-Walka
 *
 */
public class DummySynthesizer extends BaseSynthesizer {

    protected Speakable getSpeakable(String text) {
        // TODO Auto-generated method stub
        return null;
    }

    protected void handleAllocate() throws EngineStateException,
            EngineException, AudioException, SecurityException {
        // TODO Auto-generated method stub
        
    }

    protected boolean handleCancel() {
        // TODO Auto-generated method stub
        return false;
    }

    protected boolean handleCancel(int id) {
        // TODO Auto-generated method stub
        return false;
    }

    protected boolean handleCancelAll() {
        // TODO Auto-generated method stub
        return false;
    }

    protected void handleDeallocate() {
        // TODO Auto-generated method stub
    }

    protected void handlePause() {
        // TODO Auto-generated method stub
    }

    protected boolean handleResume() {
        // TODO Auto-generated method stub
        return false;
    }

    protected void handleSpeak(int id, String item) {
        // TODO Auto-generated method stub
        
    }

    protected void handleSpeak(int id, Speakable item) {
        // TODO Auto-generated method stub
        
    }

    protected AudioManager createAudioManager() {
        return new DummyAudioManager();
    }

    protected SpeechEventExecutor createSpeechEventExecutor() {
        return new DummySpeechEventExecutor();
    }

    protected VocabularyManager createVocabularyManager() {
        // TODO Auto-generated method stub
        return null;
    }

    protected EnginePropertyChangeRequestListener getChangeRequestListener() {
        // TODO Auto-generated method stub
        return null;
    }
}
