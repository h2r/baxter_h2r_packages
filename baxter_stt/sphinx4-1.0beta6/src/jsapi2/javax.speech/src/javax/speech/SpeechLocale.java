/*
 * File:    $HeadURL: https://svn.sourceforge.net/svnroot/jvoicexml/trunk/src/org/jvoicexml/Application.java$
 * Version: $LastChangedRevision: 22 $
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

//Comp. 2.0.6

public final class SpeechLocale {
    public static final SpeechLocale ENGLISH;

    public static final SpeechLocale US;
    
    public static final SpeechLocale FRENCH;

    public static final SpeechLocale GERMAN;

    
    /**
     * The default locale. Except for during bootstrapping, this should never be
     * null. Note the logic in the main constructor, to detect when
     * bootstrapping has completed.
     */
    private static SpeechLocale DEFAULT_LOCALE;

    static {
        ENGLISH = new SpeechLocale("en");
        US = new SpeechLocale("en", "US");
        FRENCH = new SpeechLocale("fr");
        GERMAN = new SpeechLocale("de");

        String defaultLanguage = System.getProperty("microedition.locale");
        if (defaultLanguage == null) {
            defaultLanguage = "en";
        }
        DEFAULT_LOCALE = new SpeechLocale(defaultLanguage);
    }

    private String language;

    private String country;

    private String variant;

    /**
     * Convert new iso639 codes to the old ones.
     * 
     * @param language
     *            the language to check
     * @return the appropriate code
     */
    private String convertLanguage(String language) {
        if (language.equals(""))
            return language;
        language = language.toLowerCase();
        int index = "he,id,yi".indexOf(language);
        if (index != -1)
            return "iw,in,ji".substring(index, index + 2);
        return language;
    }

    public SpeechLocale(String language, String country, String variant) {
        this.language = language;
        this.country = country;
        this.variant = variant;
    }

    public SpeechLocale(String language, String country) {
        this(language, country, "");
    }

    public SpeechLocale(String language) {
        this(language, "", "");
    }

    public static SpeechLocale getDefault() {
        return DEFAULT_LOCALE;
    }

    public static SpeechLocale[] getAvailableLocales() {
        return new SpeechLocale[] { ENGLISH, US, FRENCH, GERMAN };
    }

    public String getLanguage() {
        return language;
    }

    public String getCountry() {
        return country;
    }

    public String getVariant() {
        return variant;
    }

    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((country == null) ? 0 : country.hashCode());
        result = prime * result
                + ((language == null) ? 0 : language.hashCode());
        result = prime * result + ((variant == null) ? 0 : variant.hashCode());
        return result;
    }

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
        SpeechLocale other = (SpeechLocale) obj;
        if (country == null) {
            if (other.country != null) {
                return false;
            }
        } else if (!country.equals(other.country)) {
            return false;
        }
        if (language == null) {
            if (other.language != null) {
                return false;
            }
        } else if (!language.equals(other.language)) {
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

    public boolean match(SpeechLocale require) {
        if (require == null) {
            return true;
        }
        if (require.country.length() > 0) {
            if (!require.country.equals(country)) {
                return false;
            }
        }
        if (require.language.length() > 0) {
            if (!require.language.equals(language)) {
                return false;
            }
        }
        if (require.variant.length() > 0) {
            if (!require.variant.equals(variant)) {
                return false;
            }
        }
        return true;
    }
    
    public final String toString() {
        if ((language.length() == 0) && (country.length() == 0)) {
            return "";
        }

        StringBuffer str = new StringBuffer();
        str.append(language);
        if (country.length() != 0) {
            str.append('_').append(country);
        }
        if (variant.length() != 0) {
            str.append('_').append(variant);
        }

        return str.toString();
    }

}
