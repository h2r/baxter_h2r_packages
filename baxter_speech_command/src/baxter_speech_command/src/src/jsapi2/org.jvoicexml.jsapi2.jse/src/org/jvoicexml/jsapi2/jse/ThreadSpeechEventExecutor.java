/*
 * File:    $HeadURL: https://jsapi.svn.sourceforge.net/svnroot/jsapi/trunk/org.jvoicexml.jsapi2.jse/src/org/jvoicexml/jsapi2/jse/ThreadSpeechEventExecutor.java $
 * Version: $LastChangedRevision: 627 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 */
package org.jvoicexml.jsapi2.jse;

import java.util.Vector;

import org.jvoicexml.jsapi2.TerminatableSpeechEventExecutor;

/**
 * A speech event executor that is based on a thread.
 *
 * <p>
 * There is only one single thread that is responsible to execute the
 * commands asynchronously.
 * </p>
 * @author Renato Cassaca
 * @version $Revision: 627 $
 */
public final class ThreadSpeechEventExecutor
    implements TerminatableSpeechEventExecutor, Runnable {
    /** Number of msec to wait before inspecting the command queue. */
    private static final int COMMAND_POLL_INTERVALL = 1000;

    /** The thread that executes the commands. */
    private final Thread thread;

    /** Commands to execute. */
    private final Vector commands;

    /** <code>false</code> if the executor is terminating. */
    private boolean shouldRun;

    /**
     * Constructs a new object.
     */
    public ThreadSpeechEventExecutor() {
        commands = new Vector();
        thread = new Thread(this);
        shouldRun = true;
        thread.start();
    }

    /**
     * {@inheritDoc}
     *
     * Terminates the execution thread.
     */
    protected void finalize() {
        terminate();
    }

    /**
     * {@inheritDoc}
     */
    public void terminate() {
        shouldRun = false;
        synchronized (commands) {
            commands.notifyAll();
        }
    }

    /**
     * Executes the given command.
     *
     * @param command the command to execute.
     */
    public void execute(final Runnable command) {
        if (command == null) {
            throw new NullPointerException("Command must not be null!");
        }
        if (!shouldRun) {
            throw new IllegalStateException(
                    "SpeechEventExecutor is terminated!");
        }
        commands.addElement(command);
        synchronized (commands) {
            commands.notify();
        }
    }

    /**
     * {@inheritDoc}
     */
    public void run() {
        while (shouldRun) {
            while ((commands.isEmpty()) && (shouldRun)) {
                synchronized (commands) {
                    try {
                        commands.wait(COMMAND_POLL_INTERVALL);
                    } catch (InterruptedException ex) {
                        return;
                    }
                }
            }
            if (!shouldRun) {
                return;
            }

            //Use this thread to run the command
            final Runnable command = (Runnable) commands.firstElement();
            commands.removeElementAt(0);
            command.run();
        }
    }
}
