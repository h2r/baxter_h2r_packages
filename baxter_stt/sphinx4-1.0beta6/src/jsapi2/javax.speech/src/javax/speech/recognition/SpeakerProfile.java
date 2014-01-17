/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 65 $
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

package javax.speech.recognition;

//Comp 2.0.6

public class SpeakerProfile {
    public static SpeakerProfile DEFAULT = new SpeakerProfile("default", null);

    private String name;

    private String variant;

    public SpeakerProfile(String name, String variant) {
        this.name = name;
        this.variant = variant;
    }

    public String getName() {
        return name;
    }

    public String getVariant() {
        return variant;
    }
    

    /* (non-Javadoc)
     * @see java.lang.Object#hashCode()
     */
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((name == null) ? 0 : name.hashCode());
        result = prime * result + ((variant == null) ? 0 : variant.hashCode());
        return result;
    }

    /* (non-Javadoc)
     * @see java.lang.Object#equals(java.lang.Object)
     */
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        SpeakerProfile other = (SpeakerProfile) obj;
        if (name == null) {
            if (other.name != null) {
                return false;
            }
        } else if (!name.equals(other.name)) {
            return false;
        }
        if (variant == null) {
            if (other.variant != null) {
                return false;
            }
        } else if (!variant.equals(other.variant)) {
            return false;
        }
        return true;
    }

    public String toString() {
        StringBuffer str = new StringBuffer();

        str.append(getClass().getName());
        str.append("[");
        str.append(name);
        str.append(",");
        str.append(variant);
        str.append("]");

        return str.toString();
    }

    public boolean match(SpeakerProfile require) {
        if (require == null) {
            return true;
        }

        final String otherName = require.getName();
        final boolean nameMatch;
        if (otherName == null) {
            nameMatch = true;
        } else {
            nameMatch = otherName.equals(name);
        }

        final String otherVariant = require.getVariant();
        final boolean variantMatch;
        if (otherVariant == null) {
            variantMatch = true;
        } else {
            variantMatch = otherVariant.equals(variant);
        }

        return nameMatch && variantMatch;
    }
}
