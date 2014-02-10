/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/unittests/org/jvoicexml/jsapi2/jse/BaseSpeechEventExecutorTest.java $
 * Version: $LastChangedRevision: 266 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */

package org.jvoicexml.jsapi2.jse;

import java.util.Enumeration;
import java.util.Vector;

import org.jvoicexml.jsapi2.jse.ThreadSpeechEventExecutor;

import junit.framework.Assert;
import junit.framework.TestCase;

/**
 * Test cases for {@link ThreadSpeechEventExecutor}.
 * @author Dirk Schnelle-Walka
 *
 */
public final class ThreadSpeechEventExecutorTest extends TestCase {
    /** The test object. */
    private ThreadSpeechEventExecutor executor;

    /**
     * Setup the test environment.
     * @throws java.lang.Exception
     *         setup failed
     */
    public void setUp() throws Exception {
        executor = new ThreadSpeechEventExecutor();
    }

    /**
     * Cleanup of the test environment.
     * @throws java.lang.Exception
     *         cleanup failed
     */
    public void tearDown() throws Exception {
        executor.terminate();
    }

    /**
     * Test method for {@link org.jvoicexml.jsapi2.ThreadSpeechEventExecutor.BaseSpeechEventExecutor#execute(java.lang.Runnable)}.
     * @exception Exception
     *            test failed
     */
    public void testExecute() throws Exception {
        final Vector list = new Vector();
        final Runnable runnable1 = new Runnable() {
            public void run() {
                list.addElement(new Integer(1));
            }
        };
        final Runnable runnable2 = new Runnable() {
            public void run() {
                list.addElement(new Integer(2));
                synchronized (list) {
                    list.notifyAll();
                }
            }
        };
        executor.execute(runnable1);
        executor.execute(runnable2);
        synchronized (list) {
            list.wait();
        }
        Assert.assertEquals(2, list.size());
        final Enumeration enumeration = list.elements();
        Assert.assertEquals(new Integer(1), enumeration.nextElement());
        Assert.assertEquals(new Integer(2), enumeration.nextElement());
    }

}
