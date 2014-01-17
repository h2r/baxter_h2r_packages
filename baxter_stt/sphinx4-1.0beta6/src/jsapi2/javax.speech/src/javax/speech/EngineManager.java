/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 69 $
 * Date:    $LastChangedDate $
 * Author:  $LastChangedBy: schnelle $
 *
 * JSAPI - An independent reference implementation of JSR 113.
 *
 * Copyright (C) 2007 JVoiceXML group - http://jvoicexml.sourceforge.net
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

package javax.speech;

import java.io.IOException;
import java.io.InputStream;
import java.util.Enumeration;
import java.util.Vector;

import javax.speech.spi.EngineFactory;
import javax.speech.spi.EngineListFactory;

//Comp. 2.0.6

public class EngineManager {
    private static final Vector ENGINE_LIST_FACTORIES;

    private static SpeechEventExecutor executor;

    static {
        ENGINE_LIST_FACTORIES = new Vector();

        final InputStream input;
        try {
            final Class clazz = Class.forName("javax.speech.EngineManager"); 
            input = clazz.getResourceAsStream( "/speech.properties");
        } catch (ClassNotFoundException e) {
            throw new IllegalArgumentException(e.getMessage());
        }
        if (input != null) {
            final Properties props = new Properties();
            try {
                props.load(input);

                //Close input
                input.close();
            } catch (IOException e) {
                // Ignore.
            }
            
            final Enumeration keys = props.keys();
            while (keys.hasMoreElements()) {
                final String key = (String) keys.nextElement();
                final String className = (String) props.get(key);
                try {
                    registerEngineListFactory(className);
                } catch (IllegalArgumentException e) {
                    // Ignore.
                } catch (EngineException e) {
                    // Ignore.
                } catch (SecurityException e) {
                    // Ignore.
                }
            }
        }
    }

    public static EngineList availableEngines(EngineMode require) {
        final Vector modes = new Vector();

        final Enumeration enumeration = ENGINE_LIST_FACTORIES.elements();
        while (enumeration.hasMoreElements()) {
            final EngineListFactory factory = (EngineListFactory) enumeration
                    .nextElement();
            EngineList list = factory.createEngineList(require);
            if (list != null) {
                final Enumeration currentModes = list.elements();
                while (currentModes.hasMoreElements()) {
                    final EngineMode mode = (EngineMode) currentModes
                            .nextElement();
                    modes.addElement(mode);
                }
            }
        }

        EngineMode[] foundModes = new EngineMode[modes.size()];
        modes.copyInto(foundModes);

        return new EngineList(foundModes);
    }

    public static Engine createEngine(EngineMode require)
            throws IllegalArgumentException, EngineException {
        if (require == null) {
            throw new IllegalArgumentException(
                    "An engine mode must be specified to create an engine!");
        }

        final SpeechLocale defaultLocale = SpeechLocale.getDefault();
        // TODO Evaluate the default Locale
        final EngineList list = availableEngines(require);

        final Enumeration enumeration = list.elements();
        EngineFactory preferredFactory = null;
        Boolean preferredFactoryRunning = null;
        while (enumeration.hasMoreElements()) {
            final EngineMode mode = (EngineMode) enumeration.nextElement();
            if (mode instanceof EngineFactory) {
                final EngineFactory factory = (EngineFactory) mode;
                if (preferredFactory == null) {
                    preferredFactory = factory;
                    preferredFactoryRunning = mode.getRunning();
                }

                final Boolean currentFactoryRunning = mode.getRunning();
                final Boolean trueVal = new Boolean(true);
                if (trueVal.equals(currentFactoryRunning)) {
                    if (!trueVal.equals(preferredFactoryRunning)) {
                        preferredFactory = factory;
                    }
                }
            }
        }

        if (preferredFactory == null) {
            return null;
        }

        return preferredFactory.createEngine();
    }

    public static SpeechEventExecutor getSpeechEventExecutor() {
        return executor;
    }

    public static void setSpeechEventExecutor(
            SpeechEventExecutor speechEventDispatcher) {
        executor = speechEventDispatcher;
    }

    public static String getVersion() {
        return "2.0.6.0";
    }

    public static void registerEngineListFactory(String className)
            throws IllegalArgumentException, EngineException,
                SecurityException {
        final Class clazz;
        try {
            clazz = Class.forName(className);
        } catch (ClassNotFoundException e) {
            throw new IllegalArgumentException("'" + className
                    + "' cannot be loaded!");
        }

        final EngineListFactory engineListFactory;
        try {
            engineListFactory = (EngineListFactory) clazz.newInstance();
        } catch (InstantiationException e) {
            throw new IllegalArgumentException("'" + className
                    + "' cannot be created!");
        } catch (IllegalAccessException e) {
            throw new SecurityException("'" + className
                    + "' cannot be created!");
        } catch (ClassCastException e) {
            throw new IllegalArgumentException("'" + className
                    + "' does not implement EngineListFactory!");
        }

        if (!(engineListFactory instanceof EngineListFactory)) {
            throw new IllegalArgumentException("'" + className
                    + "' does not implement EngineListFactory");
        }

        final Enumeration enumeration = ENGINE_LIST_FACTORIES.elements();
        while (enumeration.hasMoreElements()) {
            final Object current = enumeration.nextElement();
            final Class currentClass = current.getClass();
            final String currentName = currentClass.getName();
            if (className.equals(currentName)) {
                return;
            }
        }

        ENGINE_LIST_FACTORIES.addElement(engineListFactory);
    }
}
