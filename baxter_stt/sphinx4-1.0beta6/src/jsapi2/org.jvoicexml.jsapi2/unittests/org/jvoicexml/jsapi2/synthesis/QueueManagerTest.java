/**
 * 
 */
package org.jvoicexml.jsapi2.synthesis;

import javax.speech.AudioSegment;
import javax.speech.synthesis.SpeakableListener;

import junit.framework.Assert;
import junit.framework.TestCase;

import org.jvoicexml.jsapi2.test.synthesis.DummySpeakableListener;
import org.jvoicexml.jsapi2.test.synthesis.DummySynthesizer;

/**
 * Test cases for {@link QueueManager}.
 * @author Dirk Schnelle-Walka
 *
 */
public final class QueueManagerTest extends TestCase {
    /** Synthesizer. */
    private BaseSynthesizer synthesizer;

    /**
     * Set up the test environment.
     */
    public void setUp() {
        synthesizer = new DummySynthesizer();
    }

    /**
     * Test method for {@link org.jvoicexml.jsapi2.jse.synthesis.QueueManager#appendItem(javax.speech.synthesis.Speakable, javax.speech.synthesis.SpeakableListener)}.
     * @exception Exception
     *            test failed.
     */
    public void testAppendItemSpeakableSpeakableListener() throws Exception {
        QueueManager manager = synthesizer.getQueueManager();
        AudioSegment segment = new AudioSegment("http://nowhere", "test");
        SpeakableListener listener = new DummySpeakableListener();
        manager.appendItem(segment, listener);
        QueueItem item = manager.getQueueItem();
        Assert.assertNotNull(item);
        Assert.assertEquals(segment, item.getAudioSegment());
        Assert.assertEquals(listener, item.getListener());
        Thread.sleep(10000);
    }

}
