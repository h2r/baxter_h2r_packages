package org.jvoicexml.jsapi2.jse.synthesis;

import java.util.logging.Logger;

import javax.sound.sampled.AudioFormat;
import javax.speech.AudioManager;
import javax.speech.SpeechEventExecutor;
import javax.speech.VocabularyManager;
import javax.speech.synthesis.Synthesizer;
import javax.speech.synthesis.SynthesizerMode;

import org.jvoicexml.jsapi2.BaseVocabularyManager;
import org.jvoicexml.jsapi2.jse.ThreadSpeechEventExecutor;
import org.jvoicexml.jsapi2.synthesis.BaseSynthesizer;


/**
 * <p>
 * Title: JSAPI 2.0
 * </p>
 *
 * <p>
 * Description: An independent reference implementation of JSR 113
 * </p>
 *
 * <p>
 * Copyright: Copyright (c) 2007
 * </p>
 *
 * <p>
 * Company: JVoiceXML group - http://jvoicexml.sourceforge.net
 * </p>
 *
 * @author Renato Cassaca
 * @author Dirk Schnelle-Walka
 * @version $Revision: $
 */
public abstract class JseBaseSynthesizer extends BaseSynthesizer
    implements Synthesizer {
    /** Logger for this class. */
    private static final Logger LOGGER =
            Logger.getLogger(JseBaseSynthesizer.class.getName());

    /**
     * Constructs a new object.
     */
    public JseBaseSynthesizer() {
        this(null);
    }

    /**
     * Constructs a new object.
     * @param engineMode the engine mode
     */
    public JseBaseSynthesizer(final SynthesizerMode engineMode) {
        super(engineMode);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected SpeechEventExecutor createSpeechEventExecutor() {
        return new ThreadSpeechEventExecutor();
    }

    /**
     * Retrieves the audio format that is produced by this synthesizer.
     * @return audio format.
     */
    protected abstract AudioFormat getAudioFormat();

    /**
     * {@inheritDoc}
     */
    @Override
    protected AudioManager createAudioManager() {
        final AudioFormat format = getAudioFormat();
        final BaseSynthesizerAudioManager manager =
            new BaseSynthesizerAudioManager(format);
        manager.setEngine(this);
        return manager;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    protected VocabularyManager createVocabularyManager() {
        return new BaseVocabularyManager();
    }
}
