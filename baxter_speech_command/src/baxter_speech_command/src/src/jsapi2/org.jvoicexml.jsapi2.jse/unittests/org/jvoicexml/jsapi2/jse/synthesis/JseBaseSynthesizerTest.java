/**
 * 
 */
package org.jvoicexml.jsapi2.jse.synthesis;

import javax.speech.AudioException;
import javax.speech.Engine;
import javax.speech.EngineException;
import javax.speech.EngineStateException;
import javax.speech.synthesis.Synthesizer;

import junit.framework.Assert;

import org.junit.Test;
import org.jvoicexml.jsapi2.jse.test.synthesis.DummySynthesizer;

/**
 * Test cases for {@link JseBaseSynthesizer}.
 * @author Dirk Schnelle-Walka
 *
 */
public class JseBaseSynthesizerTest {

    /**
     * Test method for {@link org.jvoicexml.jsapi2.BaseEngine#allocate()}.
     * @throws Exception 
     *         test failed
     */
    @Test
    public void testAllocate() throws Exception {
        final DummySynthesizer synthesizer = new DummySynthesizer();
        final long state1 = synthesizer.getEngineState();
        Assert.assertEquals(Engine.DEALLOCATED, state1);
        synthesizer.allocate();
        synthesizer.waitEngineState(Engine.ALLOCATED);
        final long state2 = synthesizer.getEngineState();
        Assert.assertEquals(Engine.ALLOCATED | Engine.PAUSED,
                Synthesizer.QUEUE_EMPTY | Engine.DEFOCUSED, state2);
    }

}
