package org.jvoicexml.jsapi2.jse.recognition;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2008</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class GrammarDefinition {

    private String grammar;
    private String name;

    public GrammarDefinition() {
        this.grammar = null;
        this.name = null;
    }

    public GrammarDefinition(String grammar, String name) {
        this.grammar = grammar;
        this.name = name;
    }

    public String getGrammar() {
        return grammar;
    }

    public String getName() {
        return name;
    }

}
