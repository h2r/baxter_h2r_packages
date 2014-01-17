/**
 * Copyright 1998-2003 Sun Microsystems, Inc.
 *
 * See the file "license.terms" for information on usage and
 * redistribution of this file, and for a DISCLAIMER OF ALL
 * WARRANTIES.
 */
package org.jvoicexml.jsapi2.jse.recognition;

import java.io.StringReader;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.speech.SpeechLocale;
import javax.speech.recognition.GrammarException;
import javax.speech.recognition.GrammarManager;
import javax.speech.recognition.Recognizer;
import javax.speech.recognition.Rule;
import javax.speech.recognition.RuleAlternatives;
import javax.speech.recognition.RuleComponent;
import javax.speech.recognition.RuleCount;
import javax.speech.recognition.RuleGrammar;
import javax.speech.recognition.RuleParse;
import javax.speech.recognition.RuleReference;
import javax.speech.recognition.RuleSequence;
import javax.speech.recognition.RuleTag;
import javax.speech.recognition.RuleToken;

import org.jvoicexml.jsapi2.recognition.BaseGrammar;

/**
 * Implementation of javax.speech.recognition.RuleGrammar.
 *
 */
public class BaseRuleGrammar extends BaseGrammar implements RuleGrammar
{
    /** Logger for this class. */
    private static final Logger LOGGER =
            Logger.getLogger(BaseRuleGrammar.class.getName());
    
    protected HashMap<String, InternalRule> rules;
    protected Vector<RuleGrammarOperation> uncommitedChanges =
        new Vector<RuleGrammarOperation>();

    //Attributes of the rule grammar
    protected String root;
    protected String version;
    protected String xmlns;
    protected String xmlBase;
    protected String mode;
    protected String tagFormat;
    protected String xmlnsXsi;
    protected String xsiSchemaLocation;
    protected String doctype;

    private int ruleId;

    protected Vector    imports;
    protected Vector    importedRules;

    /**
     * Create a new BaseRuleGrammar
     * @param recognizer the BaseRecognizer for this Grammar.
     * @param name the name of this Grammar.
     */
    public BaseRuleGrammar(final JseBaseRecognizer recognizer,
            final String name) {
        super(recognizer, name);
        rules = new HashMap<String, InternalRule>();

        //Initialize rule grammar attributes
        root = null;
        version = "1.0";
        xmlns = "";
        xmlBase = "";
        mode = "voice";
        tagFormat = "";
        xmlnsXsi = "";
        xsiSchemaLocation = "";
        doctype = null;

        ruleId = 0;
    }

    /**
     * Internal representation of a Rule that
     * holds additionally the "activable" property
     */
    private class InternalRule {
        private Rule rule;
        private boolean activable;
        private int id;
        public InternalRule(Rule r, int id) {
            rule = r;
            this.id = id;
            activable = true;
        }

        public boolean isActivable() {
            return activable;
        }

        public void setActivable(boolean status) {
            activable = status;
        }

        public boolean isPublic() {
            return rule.getScope() == Rule.PUBLIC ? true : false;
        }

        public String getRulename() { return rule.getRuleName(); }

        public Rule getRule() { return rule; }

        public int getId() { return id; }

        public String toString(){
            return rule.toString();
        }
    }

    private class InternalRuleIdComparator implements Comparator{
     public InternalRuleIdComparator() {
     }

     public int compare(Object o1, Object o2){
         InternalRule ir1 = (InternalRule)o1;
         InternalRule ir2 = (InternalRule)o2;
         return ir1.getId() > ir2.getId() ? 1 :
                 ir1.getId() < ir2.getId() ? -1 : 0;
     }
 }


    /**
     * Abstract class used to make uncommited operations uniform
     */
    private abstract class RuleGrammarOperation {
        public abstract void execute() throws GrammarException;
    }

    /**
     * Class that describes an uncommitted add rule
     */
    private class AddRuleOperation extends RuleGrammarOperation {
        private InternalRule rule;
        public AddRuleOperation(InternalRule rule) {
            this.rule = rule;
        }
        public void execute() throws GrammarException {
            if (rule != null) {
                rules.put(rule.getRulename(), rule);
            }
            else {
                throw new GrammarException("AddRule: cannot add null rules");
            }
        }
    }

    /**
     * Class that describes an uncomitted delete off a rule
     */
    private class RemoveRuleOperation extends RuleGrammarOperation {
        private String name;
        public RemoveRuleOperation(String name) {
            this.name = name;
        }
        public void execute() throws GrammarException {
            InternalRule iRule = rules.remove(name);
            if (iRule == null) {
               throw new GrammarException("Rule " + name + " was not found");
            }
            else {
                if (root == name) {
                    updateRootRule();
                }
            }
        }
    }

    /**
     * Class that describes an uncommited enable of RuleGrammar
     */
    private class GrammarEnablerOperation extends RuleGrammarOperation {
        private boolean status;
        public GrammarEnablerOperation(boolean status) {
            this.status = status;
        }
        public void execute() throws GrammarException {
            final boolean activatable = isActivatable();
            if (status != activatable) {
                setActivatable(status);
            }
        }
    }

    private class RuleEnablerOperation extends RuleGrammarOperation {
        private String[] ruleNames;
        private boolean status;
        private RuleEnablerOperation(String ruleName, boolean status) {
            ruleNames = new String[1];
            ruleNames[0] = ruleName;
            this.status = status;
        }
        private RuleEnablerOperation(String[] ruleNames, boolean status) {
            this.ruleNames = ruleNames;
            this.status = status;
        }

        public void execute() throws GrammarException {
            for (String ruleName: ruleNames) {
                InternalRule iRule = rules.get(ruleName);
                if (iRule != null) {
                    if (iRule.isPublic()) {
                        iRule.setActivable(status);
                    }
                    else {
                        throw new GrammarException("Rule: " + ruleName + " doesn't have PUBLIC scope");
                    }
                }
                else {
                    throw new GrammarException("Rule: " + ruleName + " was not found in rules");
                }
            }
        }
    }

    private class RootSetterOperation extends RuleGrammarOperation {
        private String rootRuleName;


        public RootSetterOperation(String rulename) {
            rootRuleName = rulename;

//            try {
//                throw new GrammarException("Cannot set a PRIVATE_SCOPE rule as root");
//            } catch (GrammarException e) {
//                // TODO Auto-generated catch block
//                e.printStackTrace();
//            }
            
            if(LOGGER.isLoggable(Level.FINE)){
                LOGGER.fine("Set RootRuleName : "+ rootRuleName);
            }
        }
        
        
        public void execute() throws GrammarException {
            if (rootRuleName == null) {
                updateRootRule();
            }
            else {
                final Rule rule = getRule(rootRuleName);
                if (rule == null) {
                    throw new GrammarException("Root rule '" + rootRuleName
                            + "' not found");
                }
                if (rule.getScope() == Rule.PRIVATE) {
                    throw new GrammarException("Cannot set a PRIVATE_SCOPE rule as root");
                }

                root = rootRuleName;
            }
        }
    }

//////////////////////
// Begin overridden Grammar Methods
//////////////////////
    /**
     * Set the enabled property of the Grammar.
     * From javax.speech.recognition.Grammar.
     * @param enabled the new desired state of the enabled property.
     */
   /* public void setEnabled(boolean enabled) {
        GrammarEnablerOperation geo = new GrammarEnablerOperation(enabled);
        uncommitedChanges.add(geo);
    }*/
//////////////////////
// End overridden Grammar Methods
//////////////////////

//////////////////////
// Begin RuleGrammar Methods
//////////////////////
    /**
     * Set a rule in the grammar either by creating a new rule or
     * updating an existing rule.
     * @param rule the definition of the rule.
     */
    public void addRule(final Rule rule) {
        final InternalRule iRule = new InternalRule(rule, ruleId);
        final AddRuleOperation aro = new AddRuleOperation(iRule);
        uncommitedChanges.add(aro);
        ruleId++;
    }

    /**
     * Update the root rulename
     * It defaults to the first PUBLIC_SCOPE rule that
     * is enabled
     */
    private void updateRootRule() {
        InternalRule nextRootCandidate = null;
        for (InternalRule iRule: rules.values()) {
            if (iRule.isPublic() && iRule.isActivable()) {
                if (nextRootCandidate == null) {
                    nextRootCandidate = iRule;
                }
                else {
                    if (iRule.getId() < nextRootCandidate.getId()) {
                        nextRootCandidate = iRule;
                    }
                }
            }
        }
        if (nextRootCandidate != null)
            root = nextRootCandidate.getRulename();
    }

    /**
     * Build a rule from text and add it in the grammar either by creating a new rule or
     * updating an existing rule.
     *
     * @param ruleText String
     * @throws GrammarException
     */
    public void addRule(String ruleText)  throws GrammarException {
        SrgsRuleGrammarParser srgsParser = new SrgsRuleGrammarParser();
        Rule[] rules = srgsParser.loadRule(new StringReader(ruleText));

        for (Rule r : rules){
            addRule(r);
        }
    }

    /**
     *
     * @param ruleText String
     * @throws GrammarException
     */
    public void addRules(Rule[] rules) {
        for (Rule rule: rules) {
            addRule(rule);
        }
    }

    public String getRoot() {
        if (root == null) {
            updateRootRule();
        }
        return root;
    }

    /**
     * @param rulename String
     */
    public void setRoot(String rulename) {
        final RootSetterOperation rsgo = new RootSetterOperation(rulename);
        uncommitedChanges.add(rsgo);
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("added RootRule : "+ rulename);
        }
        
    }

    protected void setAttributes(HashMap<String, String> attributes) {
        
        if(LOGGER.isLoggable(Level.FINE)){
            LOGGER.fine("Try to set Attributes for Grammar rule :" + attributes.get("root") );          
        }
        
        for (String name: attributes.keySet()) {
            setAttribute(name, attributes.get(name));
        }
    }

    public void setAttribute(String attribute, String value) throws IllegalArgumentException {
        if (attribute.equals("root")) setRoot(value);
        else if (attribute.equals("version")) version = value;
        else if (attribute.equals("xmlns")) xmlns = value;
        else if (attribute.equals("xml:lang")) setSpeechLocale(new SpeechLocale(value));
        else if (attribute.equals("xml:base")) xmlBase = value;
        else if (attribute.equals("mode")) mode = value;
        else if (attribute.equals("tag-format")) tagFormat = value;
        else if (attribute.equals("xmlns:xsi")) xmlnsXsi = value;
        else if (attribute.equals("xsi:schemaLocation")) xsiSchemaLocation = value;
        else if (attribute.equals("type") ||
                 attribute.equals("scope") ||
                 attribute.equals("src") ||
                 attribute.equals("weight") ||
                 attribute.equals("fetchtimeout") ||
                 attribute.equals("maxage") ||
                 attribute.equals("maxstale")) {
            //Ignored for VoiceXml 2.0 compatibility
            //Visit: http://www.w3.org/TR/voicexml20/vxml.dtd
        }
        else
            throw new IllegalArgumentException("Unknown atribute name: " + attribute);
    }

    public String getAttribute(String attribute) throws IllegalArgumentException {
        if (attribute.equals("root")) return getRoot();
        else if (attribute.equals("version")) return version;
        else if (attribute.equals("xmlns")) return xmlns;
        else if (attribute.equals("xml:lang")) return getSpeechLocale().toString();
        else if (attribute.equals("xml:base")) return xmlBase;
        else if (attribute.equals("mode")) return mode;
        else if (attribute.equals("tag-format")) return tagFormat;
        else if (attribute.equals("xmlns:xsi")) return xmlnsXsi;
        else if (attribute.equals("xsi:schemaLocation")) return xsiSchemaLocation;
        else
            throw new IllegalArgumentException("Unknow atribute: " + attribute);
    }

    /**
     * @todo Implement it (have to parse the param)
     * @param element String
     * @throws IllegalArgumentException
     */
    public void addElement(String element) throws IllegalArgumentException {
        throw new RuntimeException("NOT IMPLEMENTED");
    }

    /**
     * @todo Implement it (have to parse the param)
     * @param element String
     * @throws IllegalArgumentException
     */
    public void removeElement(String element) throws IllegalArgumentException {
        throw new RuntimeException("NOT IMPLEMENTED");
    }

    /**
     * @todo Implement it (have to parse the param)
     * @param element String
     * @throws IllegalArgumentException
     */
    public String[] getElements() {
        throw new RuntimeException("NOT IMPLEMENTED");
    }

    public String getDoctype() {
        return doctype;
    }

    /**
     * @todo Implement it (have to parse the param)
     * @param doctype String
     * @throws IllegalArgumentException
     */
    public void setDoctype(String doctype) throws IllegalArgumentException {
        throw new RuntimeException("NOT IMPLEMENTED");
    }


    /**
     * Return a copy of the data structure for the named rule.
     * From javax.speech.recognition.RuleGrammar.
     * @param ruleName the name of the rule.
     */
    public Rule getRule(String ruleName) {
        final InternalRule iRule = rules.get(ruleName);
        return iRule != null ?  iRule.getRule() : null;
    }

    /**
     * List the names of all rules define in this Grammar.
     * From javax.speech.recognition.RuleGrammar.
     */
    public String[] listRuleNames() {
        return rules.keySet().toArray(new String[] {});
    }

    /**
     * Delete a rule from the grammar.
     * From javax.speech.recognition.RuleGrammar.
     * @param ruleName the name of the rule.
     */
    public void removeRule(String ruleName) {
        RemoveRuleOperation rro = new RemoveRuleOperation(ruleName);
        uncommitedChanges.add(rro);
    }

    /**
     * Set the enabled state of the listed rule.
     * From javax.speech.recognition.RuleGrammar.
     * @param ruleName the name of the rule.
     * @param enabled the new enabled state.
     */
    public void setActivatable(String ruleName, boolean enabled) {
        RuleEnablerOperation reo = new RuleEnablerOperation(ruleName, enabled);
        uncommitedChanges.add(reo);
    }

    /**
     * Set the enabled state of the listed rules.
     * From javax.speech.recognition.RuleGrammar.
     * @param ruleNames the names of the rules.
     * @param enabled the new enabled state.
     */
    public void setActivatable(String[] ruleNames, boolean enabled) {
        for (String ruleName: ruleNames) {
            setActivatable(ruleName, enabled);
        }
    }

    /**
     * Return enabled state of rule.
     * From javax.speech.recognition.RuleGrammar.
     * @param ruleName the name of the rule.
     */
    public boolean isActivatable(String ruleName) {
        InternalRule iRule = rules.get(ruleName);
        return (iRule != null ? iRule.isActivable() : false);
    }

    /**
     * Resolve a simple or qualified rulename as a full rulename.
     * From javax.speech.recognition.RuleGrammar.
     * @param ruleName the name of the rule.
     */
    public RuleReference resolve(RuleReference name) throws GrammarException
    {
        RuleReference rn = new RuleReference(name.getRuleName());
/*
        String simpleName = rn.getRuleName();
        String grammarName = rn.getGrammarReference();
      //  String packageName = rn.getPackageName();
       // String fullGrammarName = rn.getFullGrammarName();
       String fullGrammarName = rn.getGrammarReference();

        // Check for badly formed RuleName
        if (packageName != null && grammarName == null) {
            throw new GrammarException("Error: badly formed rulename " + rn,
                                       null);
        }

	if (name.getRuleName().equals("NULL")) {
	  return RuleSpecial.NULL;
	}

	if (name.getRuleName().equals("VOID")) {
	  return RuleSpecial.VOID;
	}

        // Check simple case: a local rule reference
        if (fullGrammarName == null && this.getRule(simpleName) != null) {
            return new RuleReference(myName + "." + simpleName);
        }

        // Check for fully-qualified reference
        if (fullGrammarName != null) {
            RuleGrammar g = myRec.getRuleGrammar(fullGrammarName);
            if (g != null) {
                if (g.getRule(simpleName) != null) {
                    // we have a successful resolution
                    return new RuleReference(fullGrammarName + "." + simpleName);
                }
            }
        }
*/
        // Collect all matching imports into a vector.  After trying to
        // match rn to each import statement the vec will have
        // size()=0 if rn is unresolvable
        // size()=1 if rn is properly resolvable
        // size()>1 if rn is an ambiguous reference
        Vector<?> matches = new Vector();

/*
        // Get list of imports
        // Add local grammar to simply the case of checking for
        // a qualified or fully-qualified local reference.
        RuleReference imports[] = listImports();

        if (imports == null) {
            imports = new RuleReference[1];
            imports[0] = new RuleReference(getReference() + ".*");
        } else {
            RuleReference[] tmp = new RuleReference[imports.length + 1];
            System.arraycopy(imports, 0, tmp, 0, imports.length);
            tmp[imports.length] = new RuleReference(getReference() + ".*");
            imports = tmp;
        }

        // Check each import statement for a possible match
        //
        for(int i=0; i<imports.length; i++) {
            // TO-DO: update for JSAPI 1.0
            String iSimpleName = imports[i].getRuleName();
            String iGrammarName = imports[i].getGrammarReference();
            //String iPackageName = imports[i].getPackageName();
            String iFullGrammarName = imports[i].getGrammarReference();

            // Check for badly formed import name
            if (iFullGrammarName == null)
                throw new GrammarException("Error: badly formed import " +
                                           imports[i], null);

            // Get the imported grammar
            RuleGrammar gref = myRec.getRuleGrammar(iFullGrammarName);
            if (gref == null) {
                System.err.println("Warning: import of unknown grammar " +
                                   imports[i] + " in " + getReference());
                continue;
            }

            // If import includes simpleName, test that it really exists
            if (!iSimpleName.equals("*")
                && gref.getRule(iSimpleName) == null) {
                System.err.println("Warning: import of undefined rule " +
                                   imports[i] + " in " + getReference());
                continue;
            }

            // Check for fully-qualified or qualified reference
            if (iFullGrammarName.equals(fullGrammarName)
                || iGrammarName.equals(fullGrammarName)) {
                // Know that either
                //    import <ipkg.igram.???> matches <pkg.gram.???>
                // OR
                //    import <ipkg.igram.???> matches <gram.???>
                // (ipkg may be null)

                if (iSimpleName.equals("*")) {
                    if (gref.getRule(simpleName) != null) {
                        // import <pkg.gram.*> matches <pkg.gram.rulename>
                        matches.addElement(
                            new RuleReference(iFullGrammarName + "." + simpleName));
                    }
                    continue;
                } else {
                    // Now testing
                    //    import <ipkg.igram.iRuleName> against <??.gram.ruleName>
                    //
                    if (iSimpleName.equals(simpleName)) {
                        // import <pkg.gram.rulename> exact match for <???.gram.rulename>
                        matches.addElement(new RuleReference(iFullGrammarName +
                                                        "." + simpleName));
                    }
                    continue;
                }
            }

            // If we get here and rn is qualified or fully-qualified
            // then the match failed - try the next import statement
            if (fullGrammarName != null) {
                continue;
            }

            // Now test
            //    import <ipkg.igram.*> against <simpleName>

            if (iSimpleName.equals("*")) {
                if (gref.getRule(simpleName) != null) {
                    // import <pkg.gram.*> matches <simpleName>
                    matches.addElement(new RuleReference(iFullGrammarName +
                                                    "." + simpleName));
                }
                continue;
            }

            // Finally test
            //    import <ipkg.igram.iSimpleName> against <simpleName>

            if (iSimpleName.equals(simpleName)) {
                matches.addElement(new RuleReference(iFullGrammarName +
                                                "." + simpleName));
                continue;
            }
        }

        //
        // The return behavior depends upon number of matches
        //
        if (matches.size() == 0) {
            // Return null if rulename is unresolvable
            return null;
        } else if (matches.size() > 1) {
            // Throw exception if ambiguous reference
            StringBuffer b = new StringBuffer();
            b.append("Warning: ambiguous reference " + rn + " in " +
                     getReference() + " to ");
            for (int i = 0; i<matches.size(); i++) {
                RuleReference tmp = (RuleReference)(matches.elementAt(i));
                if (i > 0) {
                    b.append(" and ");
                }
                b.append(tmp);
            }
            throw new GrammarException(b.toString(), null);
        } else {*/
            // Return successfully
            return (RuleReference)(matches.elementAt(0));
       // }
    }

    /**
     * Parse the text string against the specified rule.
     * Uses the RuleParser class.
     * From javax.speech.recognition.RuleGrammar.
     * @param text the text to parse.
     * @param ruleName the name of rule to use for parsing.
     */
    public RuleParse parse(String text, String ruleName)
        throws GrammarException
    {
    /*    if (ruleName != null) {
            ruleName = stripRuleName(ruleName);
        }*/
        final Recognizer recognizer = getRecognizer();
        final GrammarManager manager = recognizer.getGrammarManager();
        final String reference = getReference();
        return RuleParser.parse(text, manager, reference, ruleName);
    }

    /**
     * Parse the tokens string against the specified rule.
     * Uses the RuleParser class.
     * From javax.speech.recognition.RuleGrammar.
     * @param tokens the tokens to parse.
     * @param ruleName the name of rule to use for parsing.
     */
    public RuleParse parse(String tokens[], String ruleName)
        throws GrammarException
    {
     /*   if (ruleName != null) {
            ruleName = stripRuleName(ruleName);
        }*/
        final Recognizer recognizer = getRecognizer();
        return RuleParser.parse(tokens, recognizer.getGrammarManager(),
                getReference(), ruleName);
    }

    /**
     * Parse the nth best result of a FinalRuleResult against the specified
     * rule.
     * Uses the RuleParser class.
     * From javax.speech.recognition.RuleGrammar.
     * @param r the FinalRuleResult.
     * @param nBest the nth best result to use.
     * @param ruleName the name of rule to use for parsing.
     */
  /*  public RuleParse parse(FinalRuleResult r, int nBest, String ruleName)
        throws GrammarException
    {
        // Some JSAPI implementations we run into are not JSAPI compliant,
        // so try a few alternatives
        ResultToken rt[] = r.getAlternativeTokens(nBest);
        if (rt != null || (rt = r.getBestTokens()) != null) {
            String tokens[] = new String[rt.length];
            for (int i=0; i<rt.length; i++) {
                tokens[i] = rt[i].getText();
            }
            return parse(tokens, ruleName);
        } else {
            return parse(r.toString(), ruleName);
        }
    }*/

    /**
     * Return a String containing the specification for this Grammar,
     * sorted by id rule
     * @param displayDisabledRules
     */
    public String toString(boolean displayDisabledRules) {
        final StringBuffer str = new StringBuffer();
        str.append("<?xml version=\"1.0\" encoding=\"iso-8859-1\"?> \n"
                  + "<!DOCTYPE grammar PUBLIC \"-//W3C//DTD GRAMMAR 1.0//EN\""
                  + " \"http://www.w3.org/TR/speech-grammar/grammar.dtd\"> \n");
        str.append("<grammar version=\"");
        str.append(version);
        str.append("\" mode=\"");
        str.append(mode);
        str.append("\" xml:lang=\"");
        str.append(getSpeechLocale());
        str.append("\" root=\"");
        str.append(getRoot());
        str.append("\" tag-format=\"");
        str.append(tagFormat);
        str.append("\" xmlns=\"http://www.w3.org/2001/06/grammar\">\n");

        Iterator it = rules.keySet().iterator();
        Vector v = new Vector();
        while(it.hasNext()){
            InternalRule r = (InternalRule) rules.get(it.next());
            v.addElement(r);
        }

        Collections.sort(v, new InternalRuleIdComparator());
        for (int i = 0; i < v.size(); ++i) {
            final InternalRule internalRule = (InternalRule) v.get(i);
            if (displayDisabledRules || internalRule.isActivable()) {
                str.append(internalRule);
                str.append('\n');
            }
        }

        str.append("</grammar>");

        return str.toString();
    }

    public String toString(){
        return toString(true);
    }
//////////////////////
// End RuleGrammar Methods
//////////////////////

//////////////////////
// NON-JSAPI METHODS
//////////////////////


    /**
     * Resolve and linkup all rule references contained in all rules.
     */
    protected void resolveAllRules()
        throws GrammarException
    {
        //StringBuffer b = new StringBuffer();

        // First make sure that all imports are resolvable
    /*    RuleReference imports[] = listImports();

        if (imports != null) {
            for(int i=0; i<imports.length; i++) {
                String gname = imports[i].getGrammarReference();
                RuleGrammar GI = myRec.getRuleGrammar(gname);
                if (GI == null) {
                    b.append("Undefined grammar " + gname +
                             " imported in " + getReference() + "\n");
                }
            }
        }
        if (b.length() > 0) {
            throw new GrammarException(b.toString(), null);
        }*/

        for (String rulename: listRuleNames()) {
            InternalRule iRule = rules.get(rulename);
            if (iRule != null) {
                resolveRule(iRule.getRule().getRuleComponent());
            }
            else {
                throw new GrammarException("null rule in Rules map!");
            }
        }
    }

    /**
     * Resolve the given rule.
     */
    protected void resolveRule(RuleComponent r)
        throws GrammarException
    {

      if (r instanceof RuleToken) {
            return;
        }

        if (r instanceof RuleAlternatives) {
            RuleAlternatives ra = (RuleAlternatives)r;
            RuleComponent array[] = ra.getRuleComponents();
            for(int i=0; i<array.length; i++) {
                resolveRule(array[i]);
            }
            return;
        }

        if (r instanceof RuleSequence) {
            RuleSequence rs = (RuleSequence)r;
            RuleComponent array[] = rs.getRuleComponents();
            for(int i=0; i<array.length; i++) {
                resolveRule(array[i]);
            }
            return;
        }

        if (r instanceof RuleCount) {
            RuleCount rc = (RuleCount) r;
            resolveRule(rc.getRuleComponent());
            return;
        }

        if (r instanceof RuleTag) {
            //RuleTag rt = (RuleTag) r;
            //resolveRule(rt.getTag());
            return;
        }

   /*    if (r instanceof BaseRuleName) {
            BaseRuleName rn = (BaseRuleName) r;
            RuleReference resolved = resolve(rn);

            if (resolved == null) {
                throw new GrammarException(
                    "Unresolvable rulename in grammar " +
                    getReference() + ": " + rn.toString(), null);
            } else {
                //[[[WDW - This forces all rule names to be fully resolved.
                //This should be changed.]]]
                rn.resolvedRuleName = resolved.getRuleName();
                rn.setRuleName(resolved.getRuleName());
                return;
            }
        }*/

        throw new GrammarException("Unknown rule type", null);
    }

    /**
     * Commits the grammar changes.
     * @exception GrammarException
     *            if there is an error in the grammar
     */
    public boolean commitChanges() throws GrammarException {
        final boolean existChanges = uncommitedChanges.size() > 0;
        RootSetterOperation rootSetter = null;
        while (uncommitedChanges.size() > 0) {
            final RuleGrammarOperation operation = uncommitedChanges.remove(0);
            if (operation instanceof RootSetterOperation) {
                rootSetter = (RootSetterOperation) operation;
            } else {
                operation.execute();
            }
        }

        // Perform the root setter as the last operation
        if (rootSetter != null) {
            rootSetter.execute();
        }
        return existChanges;
    }
}

