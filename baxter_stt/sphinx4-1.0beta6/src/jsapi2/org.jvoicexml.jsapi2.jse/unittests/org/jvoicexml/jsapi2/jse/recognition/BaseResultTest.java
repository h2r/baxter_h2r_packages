/**
 * 
 */
package org.jvoicexml.jsapi2.jse.recognition;

import javax.speech.recognition.GrammarManager;
import javax.speech.recognition.Rule;
import javax.speech.recognition.RuleComponent;
import javax.speech.recognition.RuleGrammar;
import javax.speech.recognition.RuleSequence;
import javax.speech.recognition.RuleTag;
import javax.speech.recognition.RuleToken;

import org.junit.Test;
import org.jvoicexml.jsapi2.jse.test.recognition.DummyRecognizer;


/**
 * Test cases for {@link BaseResult}.
 * @author Dirk Schnelle-Walka
 *
 */
public class BaseResultTest {

    /**
     * Test method for {@link org.jvoicexml.jsapi2.jse.recognition.BaseResult#getTags(int)}.
     * @exception Exception test failed
     */
    @Test
    public void testGetTags() throws Exception {
        final JseBaseRecognizer recognizer = new DummyRecognizer();
        final GrammarManager manager = recognizer.getGrammarManager();
        final RuleGrammar grammar =
            manager.createRuleGrammar("grammar:test", "test");
        final RuleComponent[] components = new RuleComponent[]  {
                new RuleToken("test"),
                new RuleTag("T")
        };
        final RuleSequence sequence = new RuleSequence(components);
        final Rule root = new Rule("test", sequence, Rule.PUBLIC);
        grammar.addRule(root);
        recognizer.processGrammars();
        System.out.println(grammar);
        final BaseResult result = new BaseResult(grammar, "test");
        final Object[] tags = result.getTags(1);
        System.out.println(tags[0]);
    }

}
