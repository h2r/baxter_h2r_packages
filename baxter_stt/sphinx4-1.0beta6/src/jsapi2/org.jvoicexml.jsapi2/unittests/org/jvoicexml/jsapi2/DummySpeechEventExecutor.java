/**
 * 
 */
package org.jvoicexml.jsapi2;

import javax.speech.SpeechEventExecutor;

/**
 * Dummy implementation of a speech event executor that executes synchronously.
 * @author Dirk Schnelle-Walka
 *
 */
public class DummySpeechEventExecutor implements SpeechEventExecutor {

    /**
     * {@inheritDoc}
     */
    public void execute(Runnable command) throws IllegalStateException,
            NullPointerException {
        command.run();
    }

}
