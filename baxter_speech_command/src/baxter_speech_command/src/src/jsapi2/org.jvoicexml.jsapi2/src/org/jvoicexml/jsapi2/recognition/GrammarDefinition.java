package org.jvoicexml.jsapi2.recognition;

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
    private boolean changed = false;

    public GrammarDefinition() {
        this.grammar = null;
        this.name = null;
    }

    public GrammarDefinition(String grammar, String name) {
        this(grammar, name, false);
    }

    public GrammarDefinition(String grammar, String name, boolean changed) {
        this.grammar = grammar;
        this.name = name;
        this.changed = changed;
    }

    public String getGrammar() {
        return grammar;
    }

    public String getName() {
        return name;
    }

    public boolean hasChanged() {
        return changed;
    }

}
