/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2/unittests/org/jvoicexml/jsapi2/synthesis/BaseSynthesizerPropertiesTest.java $
 * Version: $LastChangedRevision: 295 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2009 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.synthesis;

import javax.speech.EnginePropertyEvent;
import javax.speech.EnginePropertyListener;
import javax.speech.synthesis.Synthesizer;

import junit.framework.Assert;
import junit.framework.TestCase;

import org.jvoicexml.jsapi2.EnginePropertyChangeRequestEvent;
import org.jvoicexml.jsapi2.EnginePropertyChangeRequestListener;
import org.jvoicexml.jsapi2.test.synthesis.DummySynthesizer;

/**
 * Test cases for {@link BaseSynthesizerProperties}.
 * @author Dirk Schnelle-Walka
 *
 */
public final class BaseSynthesizerPropertiesTest extends TestCase
    implements EnginePropertyChangeRequestListener, EnginePropertyListener {
    /** The test object. */
    private BaseSynthesizerProperties props;

    private String name;
    private Object oldValue;
    private Object newValue;
    private EnginePropertyEvent propertyEvent;

    /**
     * Setup the test environment.
     * @throws java.lang.Exception
     *         setup failed
     */
    public void setUp() throws Exception {
        final Synthesizer synthesizer = new DummySynthesizer();
        props = new BaseSynthesizerProperties(synthesizer);
        props.addEnginePropertyChangeRequestListener(this);
        props.addEnginePropertyListener(this);
    }

    /**
     * Test method for {@link org.jvoicexml.jsapi2.synthesis.BaseSynthesizerProperties#setPitch(int)}.
     * @exception Exception
     *            test failed
     */
    public void testSetPitch() throws Exception {
        final int pitch = props.getPitch();
        final int val1 = 240;
        props.setPitch(val1);
        Assert.assertEquals(pitch, props.getPitch());
        props.commitPropertyChange(name, oldValue, newValue);
        synchronized (props) {
            props.wait(300);
        }
        Assert.assertEquals(val1, props.getPitch());
        Assert.assertNotNull(propertyEvent);
        Assert.assertEquals(BaseSynthesizerProperties.PITCH,
                propertyEvent.getPropertyName());
        Assert.assertEquals(new Integer(pitch), propertyEvent.getOldValue());
        Assert.assertEquals(new Integer(val1), propertyEvent.getNewValue());
    }

    public void propertyChangeRequest(
            final EnginePropertyChangeRequestEvent event) {
        name = event.getPropertyName();
        oldValue = event.getOldValue();
        newValue = event.getNewValue();
    }

    public void propertyUpdate(EnginePropertyEvent event) {
        propertyEvent = event;
        synchronized (props) {
            props.notifyAll();
        }
    }

}
