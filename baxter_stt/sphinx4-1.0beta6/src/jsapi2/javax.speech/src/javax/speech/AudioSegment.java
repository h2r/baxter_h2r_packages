/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 53 $
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

//Comp. 2.0.6

public class AudioSegment {
    private final String locator;

    private final String markupText;

    public AudioSegment(String locator, String markupText)
        throws IllegalArgumentException {
        if (locator == null) {
            throw new IllegalArgumentException("locator must not be null");
        }
        this.locator = locator;
        this.markupText = markupText;
    }

    public String getMediaLocator() {
        return locator;
    }

    public String getMarkupText() {
        return markupText;
    }

    public InputStream openInputStream() throws IOException, SecurityException {
        if (!isGettable()) {
            throw new SecurityException(
                    "The platform does not allow to access the input stream!");
        }
        return null;
    }

    public boolean isGettable() {
        return true;
    }

    public String toString() {
        StringBuffer str = new StringBuffer();

        str.append(getClass().getName());
        str.append("[");
        str.append(locator);
        str.append(',');
        str.append(markupText);
        str.append("]");

        return str.toString();
    }
}
