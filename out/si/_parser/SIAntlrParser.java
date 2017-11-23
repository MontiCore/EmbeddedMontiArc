// Generated from C:\Users\Delta2-PC\Desktop\Uni\SimLang\out\si\_parser\SIAntlr.g4 by ANTLR 4.5.1

package si._parser;
import de.monticore.antlr4.MCParser;

import org.antlr.v4.runtime.atn.*;
import org.antlr.v4.runtime.dfa.DFA;
import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.misc.*;
import org.antlr.v4.runtime.tree.*;
import java.util.List;
import java.util.Iterator;
import java.util.ArrayList;

@SuppressWarnings({"all", "warnings", "unchecked", "unused", "cast"})
public class SIAntlrParser extends MCParser {
	static { RuntimeMetaData.checkVersion("4.5.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		TFloatPointUnitNumber=1, THexUnitNumber=2, TUnitNumber=3, TUnitInf=4, 
		TComplexNumber=5, Name=6, WS=7, SL_COMMENT=8, ML_COMMENT=9;
	public static final int
		RULE_number_eof = 0, RULE_number = 1, RULE_floatPointUnitNumber_eof = 2, 
		RULE_floatPointUnitNumber = 3, RULE_hexUnitNumber_eof = 4, RULE_hexUnitNumber = 5, 
		RULE_unitNumber_eof = 6, RULE_unitNumber = 7, RULE_complexNumber_eof = 8, 
		RULE_complexNumber = 9;
	public static final String[] ruleNames = {
		"number_eof", "number", "floatPointUnitNumber_eof", "floatPointUnitNumber", 
		"hexUnitNumber_eof", "hexUnitNumber", "unitNumber_eof", "unitNumber", 
		"complexNumber_eof", "complexNumber"
	};

	private static final String[] _LITERAL_NAMES = {
	};
	private static final String[] _SYMBOLIC_NAMES = {
		null, "TFloatPointUnitNumber", "THexUnitNumber", "TUnitNumber", "TUnitInf", 
		"TComplexNumber", "Name", "WS", "SL_COMMENT", "ML_COMMENT"
	};
	public static final Vocabulary VOCABULARY = new VocabularyImpl(_LITERAL_NAMES, _SYMBOLIC_NAMES);

	/**
	 * @deprecated Use {@link #VOCABULARY} instead.
	 */
	@Deprecated
	public static final String[] tokenNames;
	static {
		tokenNames = new String[_SYMBOLIC_NAMES.length];
		for (int i = 0; i < tokenNames.length; i++) {
			tokenNames[i] = VOCABULARY.getLiteralName(i);
			if (tokenNames[i] == null) {
				tokenNames[i] = VOCABULARY.getSymbolicName(i);
			}

			if (tokenNames[i] == null) {
				tokenNames[i] = "<INVALID>";
			}
		}
	}

	@Override
	@Deprecated
	public String[] getTokenNames() {
		return tokenNames;
	}

	@Override

	public Vocabulary getVocabulary() {
		return VOCABULARY;
	}

	@Override
	public String getGrammarFileName() { return "SIAntlr.g4"; }

	@Override
	public String[] getRuleNames() { return ruleNames; }

	@Override
	public String getSerializedATN() { return _serializedATN; }

	@Override
	public ATN getATN() { return _ATN; }


	// Global actions

	// Convert functions
	  // convert function for TFloatPointUnitNumber
	private String convertTFloatPointUnitNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for Significant
	private String convertSignificant(Token t)  {
	    return t.getText();
	}

	  // convert function for THexUnitNumber
	private String convertTHexUnitNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for TUnitNumber
	private String convertTUnitNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for TUnitInf
	private String convertTUnitInf(Token t)  {
	    return t.getText();
	}

	  // convert function for TComplexNumber
	private String convertTComplexNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for RealNumber
	private String convertRealNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for PosNumber
	private String convertPosNumber(Token t)  {
	    return t.getText();
	}

	  // convert function for PosInt
	private String convertPosInt(Token t)  {
	    return t.getText();
	}

	  // convert function for UngroupedPosInt
	private String convertUngroupedPosInt(Token t)  {
	    return t.getText();
	}

	  // convert function for GroupedPosInt
	private String convertGroupedPosInt(Token t)  {
	    return t.getText();
	}

	  // convert function for PosIntGroup
	private String convertPosIntGroup(Token t)  {
	    return t.getText();
	}

	  // convert function for Unit
	private String convertUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for ImperialUnit
	private String convertImperialUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for OfficallyAcceptedUnit
	private String convertOfficallyAcceptedUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for SIUnit
	private String convertSIUnit(Token t)  {
	    return t.getText();
	}

	  // convert function for UnitPrefix
	private String convertUnitPrefix(Token t)  {
	    return t.getText();
	}

	  // convert function for SiUnitBaseDimension
	private String convertSiUnitBaseDimension(Token t)  {
	    return t.getText();
	}

	  // convert function for SiUnitDimensionless
	private String convertSiUnitDimensionless(Token t)  {
	    return t.getText();
	}

	  // convert function for Space
	private String convertSpace(Token t)  {
	    return t.getText();
	}

	  // convert function for NamePart
	private String convertNamePart(Token t)  {
	    return t.getText();
	}

	  // convert function for Name
	private String convertName(Token t)  {
	    return t.getText();
	}

	  // convert function for NEWLINE
	private String convertNEWLINE(Token t)  {
	    return t.getText();
	}

	  // convert function for WS
	private String convertWS(Token t)  {
	    return t.getText();
	}

	  // convert function for SL_COMMENT
	private String convertSL_COMMENT(Token t)  {
	    return t.getText();
	}

	  // convert function for ML_COMMENT
	private String convertML_COMMENT(Token t)  {
	    return t.getText();
	}


	public SIAntlrParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}
	public static class Number_eofContext extends ParserRuleContext {
		public si._ast.ASTNumber ret =  null;
		public NumberContext tmp;
		public TerminalNode EOF() { return getToken(SIAntlrParser.EOF, 0); }
		public NumberContext number() {
			return getRuleContext(NumberContext.class,0);
		}
		public Number_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_number_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitNumber_eof(this);
		}
	}

	public final Number_eofContext number_eof() throws RecognitionException {
		Number_eofContext _localctx = new Number_eofContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_number_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(20);
			((Number_eofContext)_localctx).tmp = number();
			((Number_eofContext)_localctx).ret =  ((Number_eofContext)_localctx).tmp.ret;
			setState(22);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class NumberContext extends ParserRuleContext {
		public si._ast.ASTNumber ret =  null;
		public FloatPointUnitNumberContext tmp0;
		public HexUnitNumberContext tmp1;
		public ComplexNumberContext tmp2;
		public UnitNumberContext tmp3;
		public FloatPointUnitNumberContext floatPointUnitNumber() {
			return getRuleContext(FloatPointUnitNumberContext.class,0);
		}
		public HexUnitNumberContext hexUnitNumber() {
			return getRuleContext(HexUnitNumberContext.class,0);
		}
		public ComplexNumberContext complexNumber() {
			return getRuleContext(ComplexNumberContext.class,0);
		}
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public NumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_number; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitNumber(this);
		}
	}

	public final NumberContext number() throws RecognitionException {
		NumberContext _localctx = new NumberContext(_ctx, getState());
		enterRule(_localctx, 2, RULE_number);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		si._ast.ASTNumber _aNode = null;
		_aNode=si._ast.SINodeFactory.createASTNumber();
		((NumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			setState(36);
			switch (_input.LA(1)) {
			case TFloatPointUnitNumber:
				enterOuterAlt(_localctx, 1);
				{
				setState(24);
				((NumberContext)_localctx).tmp0 = floatPointUnitNumber();
				_aNode.setFloatPointUnitNumber(_localctx.tmp0.ret);
				}
				break;
			case THexUnitNumber:
				enterOuterAlt(_localctx, 2);
				{
				setState(27);
				((NumberContext)_localctx).tmp1 = hexUnitNumber();
				_aNode.setHexUnitNumber(_localctx.tmp1.ret);
				}
				break;
			case TComplexNumber:
				enterOuterAlt(_localctx, 3);
				{
				setState(30);
				((NumberContext)_localctx).tmp2 = complexNumber();
				_aNode.setComplexNumber(_localctx.tmp2.ret);
				}
				break;
			case TUnitNumber:
				enterOuterAlt(_localctx, 4);
				{
				setState(33);
				((NumberContext)_localctx).tmp3 = unitNumber();
				_aNode.setUnitNumber(_localctx.tmp3.ret);
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class FloatPointUnitNumber_eofContext extends ParserRuleContext {
		public si._ast.ASTFloatPointUnitNumber ret =  null;
		public FloatPointUnitNumberContext tmp;
		public TerminalNode EOF() { return getToken(SIAntlrParser.EOF, 0); }
		public FloatPointUnitNumberContext floatPointUnitNumber() {
			return getRuleContext(FloatPointUnitNumberContext.class,0);
		}
		public FloatPointUnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_floatPointUnitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterFloatPointUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitFloatPointUnitNumber_eof(this);
		}
	}

	public final FloatPointUnitNumber_eofContext floatPointUnitNumber_eof() throws RecognitionException {
		FloatPointUnitNumber_eofContext _localctx = new FloatPointUnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_floatPointUnitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(38);
			((FloatPointUnitNumber_eofContext)_localctx).tmp = floatPointUnitNumber();
			((FloatPointUnitNumber_eofContext)_localctx).ret =  ((FloatPointUnitNumber_eofContext)_localctx).tmp.ret;
			setState(40);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class FloatPointUnitNumberContext extends ParserRuleContext {
		public si._ast.ASTFloatPointUnitNumber ret =  null;
		public Token tmp0;
		public TerminalNode TFloatPointUnitNumber() { return getToken(SIAntlrParser.TFloatPointUnitNumber, 0); }
		public FloatPointUnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_floatPointUnitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterFloatPointUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitFloatPointUnitNumber(this);
		}
	}

	public final FloatPointUnitNumberContext floatPointUnitNumber() throws RecognitionException {
		FloatPointUnitNumberContext _localctx = new FloatPointUnitNumberContext(_ctx, getState());
		enterRule(_localctx, 6, RULE_floatPointUnitNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		si._ast.ASTFloatPointUnitNumber _aNode = null;
		_aNode=si._ast.SINodeFactory.createASTFloatPointUnitNumber();
		((FloatPointUnitNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(42);
			((FloatPointUnitNumberContext)_localctx).tmp0 = match(TFloatPointUnitNumber);
			_aNode.setTFloatPointUnitNumber(convertTFloatPointUnitNumber(((FloatPointUnitNumberContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class HexUnitNumber_eofContext extends ParserRuleContext {
		public si._ast.ASTHexUnitNumber ret =  null;
		public HexUnitNumberContext tmp;
		public TerminalNode EOF() { return getToken(SIAntlrParser.EOF, 0); }
		public HexUnitNumberContext hexUnitNumber() {
			return getRuleContext(HexUnitNumberContext.class,0);
		}
		public HexUnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hexUnitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterHexUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitHexUnitNumber_eof(this);
		}
	}

	public final HexUnitNumber_eofContext hexUnitNumber_eof() throws RecognitionException {
		HexUnitNumber_eofContext _localctx = new HexUnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_hexUnitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(45);
			((HexUnitNumber_eofContext)_localctx).tmp = hexUnitNumber();
			((HexUnitNumber_eofContext)_localctx).ret =  ((HexUnitNumber_eofContext)_localctx).tmp.ret;
			setState(47);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class HexUnitNumberContext extends ParserRuleContext {
		public si._ast.ASTHexUnitNumber ret =  null;
		public Token tmp0;
		public TerminalNode THexUnitNumber() { return getToken(SIAntlrParser.THexUnitNumber, 0); }
		public HexUnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hexUnitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterHexUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitHexUnitNumber(this);
		}
	}

	public final HexUnitNumberContext hexUnitNumber() throws RecognitionException {
		HexUnitNumberContext _localctx = new HexUnitNumberContext(_ctx, getState());
		enterRule(_localctx, 10, RULE_hexUnitNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		si._ast.ASTHexUnitNumber _aNode = null;
		_aNode=si._ast.SINodeFactory.createASTHexUnitNumber();
		((HexUnitNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(49);
			((HexUnitNumberContext)_localctx).tmp0 = match(THexUnitNumber);
			_aNode.setTHexUnitNumber(convertTHexUnitNumber(((HexUnitNumberContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class UnitNumber_eofContext extends ParserRuleContext {
		public si._ast.ASTUnitNumber ret =  null;
		public UnitNumberContext tmp;
		public TerminalNode EOF() { return getToken(SIAntlrParser.EOF, 0); }
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public UnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitUnitNumber_eof(this);
		}
	}

	public final UnitNumber_eofContext unitNumber_eof() throws RecognitionException {
		UnitNumber_eofContext _localctx = new UnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_unitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(52);
			((UnitNumber_eofContext)_localctx).tmp = unitNumber();
			((UnitNumber_eofContext)_localctx).ret =  ((UnitNumber_eofContext)_localctx).tmp.ret;
			setState(54);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class UnitNumberContext extends ParserRuleContext {
		public si._ast.ASTUnitNumber ret =  null;
		public Token tmp0;
		public TerminalNode TUnitNumber() { return getToken(SIAntlrParser.TUnitNumber, 0); }
		public UnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitUnitNumber(this);
		}
	}

	public final UnitNumberContext unitNumber() throws RecognitionException {
		UnitNumberContext _localctx = new UnitNumberContext(_ctx, getState());
		enterRule(_localctx, 14, RULE_unitNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		si._ast.ASTUnitNumber _aNode = null;
		_aNode=si._ast.SINodeFactory.createASTUnitNumber();
		((UnitNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(56);
			((UnitNumberContext)_localctx).tmp0 = match(TUnitNumber);
			_aNode.setTUnitNumber(convertTUnitNumber(((UnitNumberContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ComplexNumber_eofContext extends ParserRuleContext {
		public si._ast.ASTComplexNumber ret =  null;
		public ComplexNumberContext tmp;
		public TerminalNode EOF() { return getToken(SIAntlrParser.EOF, 0); }
		public ComplexNumberContext complexNumber() {
			return getRuleContext(ComplexNumberContext.class,0);
		}
		public ComplexNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_complexNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterComplexNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitComplexNumber_eof(this);
		}
	}

	public final ComplexNumber_eofContext complexNumber_eof() throws RecognitionException {
		ComplexNumber_eofContext _localctx = new ComplexNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_complexNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(59);
			((ComplexNumber_eofContext)_localctx).tmp = complexNumber();
			((ComplexNumber_eofContext)_localctx).ret =  ((ComplexNumber_eofContext)_localctx).tmp.ret;
			setState(61);
			match(EOF);
			}
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static class ComplexNumberContext extends ParserRuleContext {
		public si._ast.ASTComplexNumber ret =  null;
		public Token tmp0;
		public TerminalNode TComplexNumber() { return getToken(SIAntlrParser.TComplexNumber, 0); }
		public ComplexNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_complexNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).enterComplexNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof SIAntlrListener ) ((SIAntlrListener)listener).exitComplexNumber(this);
		}
	}

	public final ComplexNumberContext complexNumber() throws RecognitionException {
		ComplexNumberContext _localctx = new ComplexNumberContext(_ctx, getState());
		enterRule(_localctx, 18, RULE_complexNumber);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		si._ast.ASTComplexNumber _aNode = null;
		_aNode=si._ast.SINodeFactory.createASTComplexNumber();
		((ComplexNumberContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(63);
			((ComplexNumberContext)_localctx).tmp0 = match(TComplexNumber);
			_aNode.setTComplexNumber(convertTComplexNumber(((ComplexNumberContext)_localctx).tmp0));
			}
			}
			_aNode.set_SourcePositionEnd(computeEndPosition(_input.LT(-1)));
		}
		catch (RecognitionException re) {
			_localctx.exception = re;
			_errHandler.reportError(this, re);
			_errHandler.recover(this, re);
		}
		finally {
			exitRule();
		}
		return _localctx;
	}

	public static final String _serializedATN =
		"\3\u0430\ud6d1\u8206\uad2d\u4417\uaef1\u8d80\uaadd\3\13E\4\2\t\2\4\3\t"+
		"\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t\13\3"+
		"\2\3\2\3\2\3\2\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\5\3\'\n"+
		"\3\3\4\3\4\3\4\3\4\3\5\3\5\3\5\3\6\3\6\3\6\3\6\3\7\3\7\3\7\3\b\3\b\3\b"+
		"\3\b\3\t\3\t\3\t\3\n\3\n\3\n\3\n\3\13\3\13\3\13\3\13\2\2\f\2\4\6\b\n\f"+
		"\16\20\22\24\2\2=\2\26\3\2\2\2\4&\3\2\2\2\6(\3\2\2\2\b,\3\2\2\2\n/\3\2"+
		"\2\2\f\63\3\2\2\2\16\66\3\2\2\2\20:\3\2\2\2\22=\3\2\2\2\24A\3\2\2\2\26"+
		"\27\5\4\3\2\27\30\b\2\1\2\30\31\7\2\2\3\31\3\3\2\2\2\32\33\5\b\5\2\33"+
		"\34\b\3\1\2\34\'\3\2\2\2\35\36\5\f\7\2\36\37\b\3\1\2\37\'\3\2\2\2 !\5"+
		"\24\13\2!\"\b\3\1\2\"\'\3\2\2\2#$\5\20\t\2$%\b\3\1\2%\'\3\2\2\2&\32\3"+
		"\2\2\2&\35\3\2\2\2& \3\2\2\2&#\3\2\2\2\'\5\3\2\2\2()\5\b\5\2)*\b\4\1\2"+
		"*+\7\2\2\3+\7\3\2\2\2,-\7\3\2\2-.\b\5\1\2.\t\3\2\2\2/\60\5\f\7\2\60\61"+
		"\b\6\1\2\61\62\7\2\2\3\62\13\3\2\2\2\63\64\7\4\2\2\64\65\b\7\1\2\65\r"+
		"\3\2\2\2\66\67\5\20\t\2\678\b\b\1\289\7\2\2\39\17\3\2\2\2:;\7\5\2\2;<"+
		"\b\t\1\2<\21\3\2\2\2=>\5\24\13\2>?\b\n\1\2?@\7\2\2\3@\23\3\2\2\2AB\7\7"+
		"\2\2BC\b\13\1\2C\25\3\2\2\2\3&";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}