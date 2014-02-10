/**
 * 
 */
package org.jvoicexml.jsapi2.jse.protocols;

import java.net.URL;

import javax.sound.sampled.AudioFormat;

import junit.framework.Assert;

import org.junit.Before;
import org.junit.Test;

/**
 * Test cases for {@link JavaSoundParser}.
 *
 * @author Dirk Schnelle-Walka
 *
 */
public class JavaSoundParserTest {
    /**
     * Set up the test environment.
     */
    @Before
    public void setUp() {
        System.setProperty("java.protocol.handler.pkgs",
                "org.jvoicexml.jsapi2.jse.protocols");
    }

    /**
     * Test method for
     * {@link net.sourceforge.gjtapi.protocols.JavaSoundParser#parse(java.net.URL)}
     * .
     *
     * @exception Exception
     *                test failed.
     */
    @Test
    public void testParse() throws Exception {
        final URL url =
            new URL("playback://audio?rate=8000&channels=2&encoding=pcm");
        AudioFormat format = JavaSoundParser.parse(url);
        Assert.assertEquals(new Float(8000.0),
                new Float(format.getSampleRate()));
        Assert.assertEquals(2, format.getChannels());
        Assert.assertEquals(AudioFormat.Encoding.PCM_SIGNED,
                format.getEncoding());
    }
}
