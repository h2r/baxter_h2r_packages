/**
 * 
 */
package org.jvoicexml.jsapi2.jse;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.InputStream;

import javax.speech.AudioSegment;

import junit.framework.Assert;

import org.junit.Test;

/**
 * Test cases for {@link BaseAudioSegment}.
 * @author Dirk Schnelle-Walka
 *
 */
public class BaseAudioSegmentTest {

    /**
     * Test method for {@link org.jvoicexml.jsapi2.jse.BaseAudioSegment#openInputStream()}.
     * @exception Exception
     *            test failed.
     */
    @Test
    public void testOpenInputStream() throws Exception {
        System.setProperty("javax.speech.supports.audio.capture", "true");
        final String test = "test";
        final ByteArrayInputStream input =
            new ByteArrayInputStream(test.getBytes());
        final AudioSegment segment1 = new BaseAudioSegment("http://localhost",
                "test", input);
        Assert.assertEquals(input, segment1.openInputStream());

        final File file = new File("build.xml");
        final AudioSegment segment2 = new BaseAudioSegment(
                file.toURI().toString(), "test");
        final InputStream input2 = segment2.openInputStream();
        Assert.assertNotNull(input2);
    }

}
