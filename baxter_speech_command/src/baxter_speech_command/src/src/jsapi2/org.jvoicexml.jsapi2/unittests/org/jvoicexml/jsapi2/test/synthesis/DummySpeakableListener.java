/**
 * 
 */
package org.jvoicexml.jsapi2.test.synthesis;

import javax.speech.synthesis.SpeakableEvent;
import javax.speech.synthesis.SpeakableListener;

/**
 * @author DS01191
 *
 */
public class DummySpeakableListener implements SpeakableListener {

    /**
     * {@inheritDoc}
     */
    public void speakableUpdate(SpeakableEvent e) {
        System.out.println(e);
    }

}
