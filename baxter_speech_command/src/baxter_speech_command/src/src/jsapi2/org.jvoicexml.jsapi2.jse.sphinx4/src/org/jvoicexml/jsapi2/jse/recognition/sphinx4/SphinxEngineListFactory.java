package org.jvoicexml.jsapi2.jse.recognition.sphinx4;

import javax.speech.EngineList;
import javax.speech.EngineMode;
import javax.speech.recognition.RecognizerMode;
import javax.speech.spi.EngineListFactory;


/**
 * <p>Title: JSAPI 2.0</p>
 *
 * <p>Description: An independent reference implementation of JSR 113</p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: JVoiceXML group - http://jvoicexml.sourceforge.net</p>
 *
 * @author Renato Cassaca
 * @version 1.0
 */
public class SphinxEngineListFactory implements EngineListFactory {
    /** Available engined modes. */
    private final EngineMode[] engineModes;

    /**
     * Constructs a new object.
     */
    public SphinxEngineListFactory() {
        engineModes = new EngineMode[] {new SphinxRecognizerMode()};
    }

    /**
     * {@inheritDoc}
     */
    public EngineList createEngineList(final EngineMode require) {
        // Must be a recognizer.
        if (!(require instanceof RecognizerMode)) {
            return null;
        }
        EngineList engineList = new EngineList(engineModes);

        if (require != null) {
            //Removes unwanted modes
            engineList.requireMatch(require);
        }

        return (engineList.size() > 0 ? engineList : null);
    }
}
