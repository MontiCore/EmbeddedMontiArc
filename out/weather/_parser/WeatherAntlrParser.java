// Generated from C:\Users\Delta2-PC\Desktop\Uni\SimLang\out\weather\_parser\WeatherAntlr.g4 by ANTLR 4.5.1

package weather._parser;
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
public class WeatherAntlrParser extends MCParser {
	static { RuntimeMetaData.checkVersion("4.5.1", RuntimeMetaData.VERSION); }

	protected static final DFA[] _decisionToDFA;
	protected static final PredictionContextCache _sharedContextCache =
		new PredictionContextCache();
	public static final int
		T__0=1, T__1=2, T__2=3, T__3=4, T__4=5, T__5=6, T__6=7, T__7=8, T__8=9, 
		T__9=10, T__10=11, T__11=12, T__12=13, T__13=14, T__14=15, T__15=16, T__16=17, 
		T__17=18, T__18=19, T__19=20, T__20=21, T__21=22, T__22=23, T__23=24, 
		T__24=25, T__25=26, T__26=27, T__27=28, CONTRAILS=29, SNAIN=30, ALTOSTRATUS=31, 
		SMOG=32, LANDSPOUT=33, NONE=34, NIMBOSTRATUS=35, CUMULONIMBUS=36, STRATUS=37, 
		STRATOCUMULUS=38, MIRAGE=39, TEMPERATURE=40, SIGHT=41, HUMIDITY=42, WATERSPOUT=43, 
		SLEET=44, NOCTILUCENT=45, GRAUPEL=46, RAIN=47, ALTOCUMULUS=48, UNLIMITED=49, 
		CIRRUS=50, GUSTNADO=51, LPAREN=52, RPAREN=53, PRESSURE=54, DRIZZLE=55, 
		HAIL=56, WINDDIRECTION=57, CIRROCUMULUS=58, RAINBOW=59, CLOUDING=60, SNOW=61, 
		WINDSTRENGTH=62, CIRROSTRATUS=63, THUNDERSTORM=64, FOG=65, Numb=66, TFloatPointUnitNumber=67, 
		THexUnitNumber=68, TUnitNumber=69, TUnitInf=70, TComplexNumber=71, Name=72, 
		WS=73, SL_COMMENT=74, ML_COMMENT=75;
	public static final int
		RULE_number_eof = 0, RULE_number = 1, RULE_floatPointUnitNumber_eof = 2, 
		RULE_floatPointUnitNumber = 3, RULE_hexUnitNumber_eof = 4, RULE_hexUnitNumber = 5, 
		RULE_unitNumber_eof = 6, RULE_unitNumber = 7, RULE_complexNumber_eof = 8, 
		RULE_complexNumber = 9, RULE_temperature_eof = 10, RULE_temperature = 11, 
		RULE_humidity_eof = 12, RULE_humidity = 13, RULE_pressure_eof = 14, RULE_pressure = 15, 
		RULE_windstrength_eof = 16, RULE_windstrength = 17, RULE_winddirection_eof = 18, 
		RULE_winddirection = 19, RULE_precipitationtype_eof = 20, RULE_precipitationtype = 21, 
		RULE_precipitationamount_eof = 22, RULE_precipitationamount = 23, RULE_clouding_eof = 24, 
		RULE_clouding = 25, RULE_sight_eof = 26, RULE_sight = 27, RULE_weatherPhenomena_eof = 28, 
		RULE_weatherPhenomena = 29, RULE_opticalPhenomena_eof = 30, RULE_opticalPhenomena = 31, 
		RULE_artificialPhenomena_eof = 32, RULE_artificialPhenomena = 33;
	public static final String[] ruleNames = {
		"number_eof", "number", "floatPointUnitNumber_eof", "floatPointUnitNumber", 
		"hexUnitNumber_eof", "hexUnitNumber", "unitNumber_eof", "unitNumber", 
		"complexNumber_eof", "complexNumber", "temperature_eof", "temperature", 
		"humidity_eof", "humidity", "pressure_eof", "pressure", "windstrength_eof", 
		"windstrength", "winddirection_eof", "winddirection", "precipitationtype_eof", 
		"precipitationtype", "precipitationamount_eof", "precipitationamount", 
		"clouding_eof", "clouding", "sight_eof", "sight", "weatherPhenomena_eof", 
		"weatherPhenomena", "opticalPhenomena_eof", "opticalPhenomena", "artificialPhenomena_eof", 
		"artificialPhenomena"
	};

	private static final String[] _LITERAL_NAMES = {
		null, "'precipitation_type'", "'freezing drizzle'", "'freezing rain'", 
		"'snow rain'", "'snow grains'", "'ice pellets'", "'snow pellets'", "'ice crystals'", 
		"'precipitation_amount'", "'polar stratospheric'", "'cumulus humilis'", 
		"'cumulus mediocris'", "'cumulus congestus'", "'weather_phenomena'", "'rope tornado'", 
		"'cone tornado'", "'wedge tornado'", "'multi-vortex tornado'", "'dust devil'", 
		"'steam devil'", "'optical_phenomena'", "'northern lights'", "'circumzenithal arc'", 
		"'zodiacal light'", "'crepuscular rays'", "'fog bow'", "'artificial_phenomena'", 
		"'rocket exhaust trails'", "'contrails'", "'snain'", "'altostratus'", 
		"'smog'", "'landspout'", "'none'", "'nimbostratus'", "'cumulonimbus'", 
		"'stratus'", "'stratocumulus'", "'mirage'", "'temperature'", "'sight'", 
		"'humidity'", "'waterspout'", "'sleet'", "'noctilucent'", "'graupel'", 
		"'rain'", "'altocumulus'", "'unlimited'", "'cirrus'", "'gustnado'", "'('", 
		"')'", "'pressure'", "'drizzle'", "'hail'", "'winddirection'", "'cirrocumulus'", 
		"'rainbow'", "'clouding'", "'snow'", "'windstrength'", "'cirrostratus'", 
		"'thunderstorm'", "'fog'"
	};
	private static final String[] _SYMBOLIC_NAMES = {
		null, null, null, null, null, null, null, null, null, null, null, null, 
		null, null, null, null, null, null, null, null, null, null, null, null, 
		null, null, null, null, null, "CONTRAILS", "SNAIN", "ALTOSTRATUS", "SMOG", 
		"LANDSPOUT", "NONE", "NIMBOSTRATUS", "CUMULONIMBUS", "STRATUS", "STRATOCUMULUS", 
		"MIRAGE", "TEMPERATURE", "SIGHT", "HUMIDITY", "WATERSPOUT", "SLEET", "NOCTILUCENT", 
		"GRAUPEL", "RAIN", "ALTOCUMULUS", "UNLIMITED", "CIRRUS", "GUSTNADO", "LPAREN", 
		"RPAREN", "PRESSURE", "DRIZZLE", "HAIL", "WINDDIRECTION", "CIRROCUMULUS", 
		"RAINBOW", "CLOUDING", "SNOW", "WINDSTRENGTH", "CIRROSTRATUS", "THUNDERSTORM", 
		"FOG", "Numb", "TFloatPointUnitNumber", "THexUnitNumber", "TUnitNumber", 
		"TUnitInf", "TComplexNumber", "Name", "WS", "SL_COMMENT", "ML_COMMENT"
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
	public String getGrammarFileName() { return "WeatherAntlr.g4"; }

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

	  // convert function for Numb
	private String convertNumb(Token t)  {
	    return t.getText();
	}


	public WeatherAntlrParser(TokenStream input) {
		super(input);
		_interp = new ParserATNSimulator(this,_ATN,_decisionToDFA,_sharedContextCache);
	}
	public static class Number_eofContext extends ParserRuleContext {
		public si._ast.ASTNumber ret =  null;
		public NumberContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public NumberContext number() {
			return getRuleContext(NumberContext.class,0);
		}
		public Number_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_number_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitNumber_eof(this);
		}
	}

	public final Number_eofContext number_eof() throws RecognitionException {
		Number_eofContext _localctx = new Number_eofContext(_ctx, getState());
		enterRule(_localctx, 0, RULE_number_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(68);
			((Number_eofContext)_localctx).tmp = number();
			((Number_eofContext)_localctx).ret =  ((Number_eofContext)_localctx).tmp.ret;
			setState(70);
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
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitNumber(this);
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
			setState(84);
			switch (_input.LA(1)) {
			case TFloatPointUnitNumber:
				enterOuterAlt(_localctx, 1);
				{
				setState(72);
				((NumberContext)_localctx).tmp0 = floatPointUnitNumber();
				_aNode.setFloatPointUnitNumber(_localctx.tmp0.ret);
				}
				break;
			case THexUnitNumber:
				enterOuterAlt(_localctx, 2);
				{
				setState(75);
				((NumberContext)_localctx).tmp1 = hexUnitNumber();
				_aNode.setHexUnitNumber(_localctx.tmp1.ret);
				}
				break;
			case TComplexNumber:
				enterOuterAlt(_localctx, 3);
				{
				setState(78);
				((NumberContext)_localctx).tmp2 = complexNumber();
				_aNode.setComplexNumber(_localctx.tmp2.ret);
				}
				break;
			case TUnitNumber:
				enterOuterAlt(_localctx, 4);
				{
				setState(81);
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
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public FloatPointUnitNumberContext floatPointUnitNumber() {
			return getRuleContext(FloatPointUnitNumberContext.class,0);
		}
		public FloatPointUnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_floatPointUnitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterFloatPointUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitFloatPointUnitNumber_eof(this);
		}
	}

	public final FloatPointUnitNumber_eofContext floatPointUnitNumber_eof() throws RecognitionException {
		FloatPointUnitNumber_eofContext _localctx = new FloatPointUnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 4, RULE_floatPointUnitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(86);
			((FloatPointUnitNumber_eofContext)_localctx).tmp = floatPointUnitNumber();
			((FloatPointUnitNumber_eofContext)_localctx).ret =  ((FloatPointUnitNumber_eofContext)_localctx).tmp.ret;
			setState(88);
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
		public TerminalNode TFloatPointUnitNumber() { return getToken(WeatherAntlrParser.TFloatPointUnitNumber, 0); }
		public FloatPointUnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_floatPointUnitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterFloatPointUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitFloatPointUnitNumber(this);
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
			setState(90);
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
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public HexUnitNumberContext hexUnitNumber() {
			return getRuleContext(HexUnitNumberContext.class,0);
		}
		public HexUnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hexUnitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHexUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHexUnitNumber_eof(this);
		}
	}

	public final HexUnitNumber_eofContext hexUnitNumber_eof() throws RecognitionException {
		HexUnitNumber_eofContext _localctx = new HexUnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 8, RULE_hexUnitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(93);
			((HexUnitNumber_eofContext)_localctx).tmp = hexUnitNumber();
			((HexUnitNumber_eofContext)_localctx).ret =  ((HexUnitNumber_eofContext)_localctx).tmp.ret;
			setState(95);
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
		public TerminalNode THexUnitNumber() { return getToken(WeatherAntlrParser.THexUnitNumber, 0); }
		public HexUnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_hexUnitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHexUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHexUnitNumber(this);
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
			setState(97);
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
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public UnitNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unitNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterUnitNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitUnitNumber_eof(this);
		}
	}

	public final UnitNumber_eofContext unitNumber_eof() throws RecognitionException {
		UnitNumber_eofContext _localctx = new UnitNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 12, RULE_unitNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(100);
			((UnitNumber_eofContext)_localctx).tmp = unitNumber();
			((UnitNumber_eofContext)_localctx).ret =  ((UnitNumber_eofContext)_localctx).tmp.ret;
			setState(102);
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
		public TerminalNode TUnitNumber() { return getToken(WeatherAntlrParser.TUnitNumber, 0); }
		public UnitNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_unitNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterUnitNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitUnitNumber(this);
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
			setState(104);
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
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public ComplexNumberContext complexNumber() {
			return getRuleContext(ComplexNumberContext.class,0);
		}
		public ComplexNumber_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_complexNumber_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterComplexNumber_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitComplexNumber_eof(this);
		}
	}

	public final ComplexNumber_eofContext complexNumber_eof() throws RecognitionException {
		ComplexNumber_eofContext _localctx = new ComplexNumber_eofContext(_ctx, getState());
		enterRule(_localctx, 16, RULE_complexNumber_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(107);
			((ComplexNumber_eofContext)_localctx).tmp = complexNumber();
			((ComplexNumber_eofContext)_localctx).ret =  ((ComplexNumber_eofContext)_localctx).tmp.ret;
			setState(109);
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
		public TerminalNode TComplexNumber() { return getToken(WeatherAntlrParser.TComplexNumber, 0); }
		public ComplexNumberContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_complexNumber; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterComplexNumber(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitComplexNumber(this);
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
			setState(111);
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

	public static class Temperature_eofContext extends ParserRuleContext {
		public weather._ast.ASTTemperature ret =  null;
		public TemperatureContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public TemperatureContext temperature() {
			return getRuleContext(TemperatureContext.class,0);
		}
		public Temperature_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_temperature_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterTemperature_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitTemperature_eof(this);
		}
	}

	public final Temperature_eofContext temperature_eof() throws RecognitionException {
		Temperature_eofContext _localctx = new Temperature_eofContext(_ctx, getState());
		enterRule(_localctx, 20, RULE_temperature_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(114);
			((Temperature_eofContext)_localctx).tmp = temperature();
			((Temperature_eofContext)_localctx).ret =  ((Temperature_eofContext)_localctx).tmp.ret;
			setState(116);
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

	public static class TemperatureContext extends ParserRuleContext {
		public weather._ast.ASTTemperature ret =  null;
		public UnitNumberContext tmp0;
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public TerminalNode TEMPERATURE() { return getToken(WeatherAntlrParser.TEMPERATURE, 0); }
		public TemperatureContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_temperature; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterTemperature(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitTemperature(this);
		}
	}

	public final TemperatureContext temperature() throws RecognitionException {
		TemperatureContext _localctx = new TemperatureContext(_ctx, getState());
		enterRule(_localctx, 22, RULE_temperature);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTTemperature _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTTemperature();
		((TemperatureContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(118);
			match(TEMPERATURE);
			}
			setState(119);
			((TemperatureContext)_localctx).tmp0 = unitNumber();
			_aNode.setWeatherTemperature(_localctx.tmp0.ret);
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

	public static class Humidity_eofContext extends ParserRuleContext {
		public weather._ast.ASTHumidity ret =  null;
		public HumidityContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public HumidityContext humidity() {
			return getRuleContext(HumidityContext.class,0);
		}
		public Humidity_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_humidity_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHumidity_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHumidity_eof(this);
		}
	}

	public final Humidity_eofContext humidity_eof() throws RecognitionException {
		Humidity_eofContext _localctx = new Humidity_eofContext(_ctx, getState());
		enterRule(_localctx, 24, RULE_humidity_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(122);
			((Humidity_eofContext)_localctx).tmp = humidity();
			((Humidity_eofContext)_localctx).ret =  ((Humidity_eofContext)_localctx).tmp.ret;
			setState(124);
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

	public static class HumidityContext extends ParserRuleContext {
		public weather._ast.ASTHumidity ret =  null;
		public UnitNumberContext tmp0;
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public TerminalNode HUMIDITY() { return getToken(WeatherAntlrParser.HUMIDITY, 0); }
		public HumidityContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_humidity; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterHumidity(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitHumidity(this);
		}
	}

	public final HumidityContext humidity() throws RecognitionException {
		HumidityContext _localctx = new HumidityContext(_ctx, getState());
		enterRule(_localctx, 26, RULE_humidity);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTHumidity _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTHumidity();
		((HumidityContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(126);
			match(HUMIDITY);
			}
			setState(127);
			((HumidityContext)_localctx).tmp0 = unitNumber();
			_aNode.setWeatherHumidity(_localctx.tmp0.ret);
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

	public static class Pressure_eofContext extends ParserRuleContext {
		public weather._ast.ASTPressure ret =  null;
		public PressureContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public PressureContext pressure() {
			return getRuleContext(PressureContext.class,0);
		}
		public Pressure_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_pressure_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPressure_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPressure_eof(this);
		}
	}

	public final Pressure_eofContext pressure_eof() throws RecognitionException {
		Pressure_eofContext _localctx = new Pressure_eofContext(_ctx, getState());
		enterRule(_localctx, 28, RULE_pressure_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(130);
			((Pressure_eofContext)_localctx).tmp = pressure();
			((Pressure_eofContext)_localctx).ret =  ((Pressure_eofContext)_localctx).tmp.ret;
			setState(132);
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

	public static class PressureContext extends ParserRuleContext {
		public weather._ast.ASTPressure ret =  null;
		public UnitNumberContext tmp0;
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public TerminalNode PRESSURE() { return getToken(WeatherAntlrParser.PRESSURE, 0); }
		public PressureContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_pressure; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPressure(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPressure(this);
		}
	}

	public final PressureContext pressure() throws RecognitionException {
		PressureContext _localctx = new PressureContext(_ctx, getState());
		enterRule(_localctx, 30, RULE_pressure);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTPressure _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTPressure();
		((PressureContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(134);
			match(PRESSURE);
			}
			setState(135);
			((PressureContext)_localctx).tmp0 = unitNumber();
			_aNode.setWeatherPressure(_localctx.tmp0.ret);
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

	public static class Windstrength_eofContext extends ParserRuleContext {
		public weather._ast.ASTWindstrength ret =  null;
		public WindstrengthContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public WindstrengthContext windstrength() {
			return getRuleContext(WindstrengthContext.class,0);
		}
		public Windstrength_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_windstrength_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWindstrength_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWindstrength_eof(this);
		}
	}

	public final Windstrength_eofContext windstrength_eof() throws RecognitionException {
		Windstrength_eofContext _localctx = new Windstrength_eofContext(_ctx, getState());
		enterRule(_localctx, 32, RULE_windstrength_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(138);
			((Windstrength_eofContext)_localctx).tmp = windstrength();
			((Windstrength_eofContext)_localctx).ret =  ((Windstrength_eofContext)_localctx).tmp.ret;
			setState(140);
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

	public static class WindstrengthContext extends ParserRuleContext {
		public weather._ast.ASTWindstrength ret =  null;
		public UnitNumberContext tmp0;
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public TerminalNode WINDSTRENGTH() { return getToken(WeatherAntlrParser.WINDSTRENGTH, 0); }
		public WindstrengthContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_windstrength; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWindstrength(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWindstrength(this);
		}
	}

	public final WindstrengthContext windstrength() throws RecognitionException {
		WindstrengthContext _localctx = new WindstrengthContext(_ctx, getState());
		enterRule(_localctx, 34, RULE_windstrength);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTWindstrength _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTWindstrength();
		((WindstrengthContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(142);
			match(WINDSTRENGTH);
			}
			setState(143);
			((WindstrengthContext)_localctx).tmp0 = unitNumber();
			_aNode.setWeatherWindstrength(_localctx.tmp0.ret);
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

	public static class Winddirection_eofContext extends ParserRuleContext {
		public weather._ast.ASTWinddirection ret =  null;
		public WinddirectionContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public WinddirectionContext winddirection() {
			return getRuleContext(WinddirectionContext.class,0);
		}
		public Winddirection_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_winddirection_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWinddirection_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWinddirection_eof(this);
		}
	}

	public final Winddirection_eofContext winddirection_eof() throws RecognitionException {
		Winddirection_eofContext _localctx = new Winddirection_eofContext(_ctx, getState());
		enterRule(_localctx, 36, RULE_winddirection_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(146);
			((Winddirection_eofContext)_localctx).tmp = winddirection();
			((Winddirection_eofContext)_localctx).ret =  ((Winddirection_eofContext)_localctx).tmp.ret;
			setState(148);
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

	public static class WinddirectionContext extends ParserRuleContext {
		public weather._ast.ASTWinddirection ret =  null;
		public UnitNumberContext tmp0;
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public TerminalNode WINDDIRECTION() { return getToken(WeatherAntlrParser.WINDDIRECTION, 0); }
		public WinddirectionContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_winddirection; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWinddirection(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWinddirection(this);
		}
	}

	public final WinddirectionContext winddirection() throws RecognitionException {
		WinddirectionContext _localctx = new WinddirectionContext(_ctx, getState());
		enterRule(_localctx, 38, RULE_winddirection);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTWinddirection _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTWinddirection();
		((WinddirectionContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(150);
			match(WINDDIRECTION);
			}
			setState(151);
			((WinddirectionContext)_localctx).tmp0 = unitNumber();
			_aNode.setWeatherWinddirection(_localctx.tmp0.ret);
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

	public static class Precipitationtype_eofContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationtype ret =  null;
		public PrecipitationtypeContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public PrecipitationtypeContext precipitationtype() {
			return getRuleContext(PrecipitationtypeContext.class,0);
		}
		public Precipitationtype_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationtype_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationtype_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationtype_eof(this);
		}
	}

	public final Precipitationtype_eofContext precipitationtype_eof() throws RecognitionException {
		Precipitationtype_eofContext _localctx = new Precipitationtype_eofContext(_ctx, getState());
		enterRule(_localctx, 40, RULE_precipitationtype_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(154);
			((Precipitationtype_eofContext)_localctx).tmp = precipitationtype();
			((Precipitationtype_eofContext)_localctx).ret =  ((Precipitationtype_eofContext)_localctx).tmp.ret;
			setState(156);
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

	public static class PrecipitationtypeContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationtype ret =  null;
		public TerminalNode NONE() { return getToken(WeatherAntlrParser.NONE, 0); }
		public TerminalNode DRIZZLE() { return getToken(WeatherAntlrParser.DRIZZLE, 0); }
		public TerminalNode RAIN() { return getToken(WeatherAntlrParser.RAIN, 0); }
		public TerminalNode SNAIN() { return getToken(WeatherAntlrParser.SNAIN, 0); }
		public TerminalNode SNOW() { return getToken(WeatherAntlrParser.SNOW, 0); }
		public TerminalNode SLEET() { return getToken(WeatherAntlrParser.SLEET, 0); }
		public TerminalNode HAIL() { return getToken(WeatherAntlrParser.HAIL, 0); }
		public TerminalNode GRAUPEL() { return getToken(WeatherAntlrParser.GRAUPEL, 0); }
		public PrecipitationtypeContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationtype; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationtype(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationtype(this);
		}
	}

	public final PrecipitationtypeContext precipitationtype() throws RecognitionException {
		PrecipitationtypeContext _localctx = new PrecipitationtypeContext(_ctx, getState());
		enterRule(_localctx, 42, RULE_precipitationtype);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTPrecipitationtype _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTPrecipitationtype();
		((PrecipitationtypeContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(158);
			match(T__0);
			}
			setState(174);
			switch (_input.LA(1)) {
			case NONE:
				{
				{
				setState(159);
				match(NONE);
				}
				}
				break;
			case DRIZZLE:
				{
				{
				setState(160);
				match(DRIZZLE);
				}
				}
				break;
			case RAIN:
				{
				{
				setState(161);
				match(RAIN);
				}
				}
				break;
			case T__1:
				{
				{
				setState(162);
				match(T__1);
				}
				}
				break;
			case T__2:
				{
				{
				setState(163);
				match(T__2);
				}
				}
				break;
			case T__3:
				{
				{
				setState(164);
				match(T__3);
				}
				}
				break;
			case SNAIN:
				{
				{
				setState(165);
				match(SNAIN);
				}
				}
				break;
			case SNOW:
				{
				{
				setState(166);
				match(SNOW);
				}
				}
				break;
			case T__4:
				{
				{
				setState(167);
				match(T__4);
				}
				}
				break;
			case T__5:
				{
				{
				setState(168);
				match(T__5);
				}
				}
				break;
			case SLEET:
				{
				{
				setState(169);
				match(SLEET);
				}
				}
				break;
			case HAIL:
				{
				{
				setState(170);
				match(HAIL);
				}
				}
				break;
			case T__6:
				{
				{
				setState(171);
				match(T__6);
				}
				}
				break;
			case GRAUPEL:
				{
				{
				setState(172);
				match(GRAUPEL);
				}
				}
				break;
			case T__7:
				{
				{
				setState(173);
				match(T__7);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
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

	public static class Precipitationamount_eofContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationamount ret =  null;
		public PrecipitationamountContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public PrecipitationamountContext precipitationamount() {
			return getRuleContext(PrecipitationamountContext.class,0);
		}
		public Precipitationamount_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationamount_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationamount_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationamount_eof(this);
		}
	}

	public final Precipitationamount_eofContext precipitationamount_eof() throws RecognitionException {
		Precipitationamount_eofContext _localctx = new Precipitationamount_eofContext(_ctx, getState());
		enterRule(_localctx, 44, RULE_precipitationamount_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(176);
			((Precipitationamount_eofContext)_localctx).tmp = precipitationamount();
			((Precipitationamount_eofContext)_localctx).ret =  ((Precipitationamount_eofContext)_localctx).tmp.ret;
			setState(178);
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

	public static class PrecipitationamountContext extends ParserRuleContext {
		public weather._ast.ASTPrecipitationamount ret =  null;
		public UnitNumberContext tmp0;
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public PrecipitationamountContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_precipitationamount; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterPrecipitationamount(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitPrecipitationamount(this);
		}
	}

	public final PrecipitationamountContext precipitationamount() throws RecognitionException {
		PrecipitationamountContext _localctx = new PrecipitationamountContext(_ctx, getState());
		enterRule(_localctx, 46, RULE_precipitationamount);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTPrecipitationamount _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTPrecipitationamount();
		((PrecipitationamountContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(180);
			match(T__8);
			}
			setState(181);
			((PrecipitationamountContext)_localctx).tmp0 = unitNumber();
			_aNode.setWeatherPrecipitationamount(_localctx.tmp0.ret);
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

	public static class Clouding_eofContext extends ParserRuleContext {
		public weather._ast.ASTClouding ret =  null;
		public CloudingContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public CloudingContext clouding() {
			return getRuleContext(CloudingContext.class,0);
		}
		public Clouding_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_clouding_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterClouding_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitClouding_eof(this);
		}
	}

	public final Clouding_eofContext clouding_eof() throws RecognitionException {
		Clouding_eofContext _localctx = new Clouding_eofContext(_ctx, getState());
		enterRule(_localctx, 48, RULE_clouding_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(184);
			((Clouding_eofContext)_localctx).tmp = clouding();
			((Clouding_eofContext)_localctx).ret =  ((Clouding_eofContext)_localctx).tmp.ret;
			setState(186);
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

	public static class CloudingContext extends ParserRuleContext {
		public weather._ast.ASTClouding ret =  null;
		public TerminalNode CLOUDING() { return getToken(WeatherAntlrParser.CLOUDING, 0); }
		public TerminalNode CIRROSTRATUS() { return getToken(WeatherAntlrParser.CIRROSTRATUS, 0); }
		public TerminalNode ALTOSTRATUS() { return getToken(WeatherAntlrParser.ALTOSTRATUS, 0); }
		public TerminalNode STRATUS() { return getToken(WeatherAntlrParser.STRATUS, 0); }
		public TerminalNode NIMBOSTRATUS() { return getToken(WeatherAntlrParser.NIMBOSTRATUS, 0); }
		public TerminalNode NOCTILUCENT() { return getToken(WeatherAntlrParser.NOCTILUCENT, 0); }
		public TerminalNode CIRRUS() { return getToken(WeatherAntlrParser.CIRRUS, 0); }
		public TerminalNode CIRROCUMULUS() { return getToken(WeatherAntlrParser.CIRROCUMULUS, 0); }
		public TerminalNode ALTOCUMULUS() { return getToken(WeatherAntlrParser.ALTOCUMULUS, 0); }
		public TerminalNode STRATOCUMULUS() { return getToken(WeatherAntlrParser.STRATOCUMULUS, 0); }
		public TerminalNode CUMULONIMBUS() { return getToken(WeatherAntlrParser.CUMULONIMBUS, 0); }
		public TerminalNode NONE() { return getToken(WeatherAntlrParser.NONE, 0); }
		public CloudingContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_clouding; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterClouding(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitClouding(this);
		}
	}

	public final CloudingContext clouding() throws RecognitionException {
		CloudingContext _localctx = new CloudingContext(_ctx, getState());
		enterRule(_localctx, 50, RULE_clouding);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTClouding _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTClouding();
		((CloudingContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(188);
			match(CLOUDING);
			}
			setState(204);
			switch (_input.LA(1)) {
			case CIRROSTRATUS:
				{
				{
				setState(189);
				match(CIRROSTRATUS);
				}
				}
				break;
			case ALTOSTRATUS:
				{
				{
				setState(190);
				match(ALTOSTRATUS);
				}
				{
				setState(191);
				match(STRATUS);
				}
				}
				break;
			case NIMBOSTRATUS:
				{
				{
				setState(192);
				match(NIMBOSTRATUS);
				}
				}
				break;
			case NOCTILUCENT:
				{
				{
				setState(193);
				match(NOCTILUCENT);
				}
				}
				break;
			case T__9:
				{
				{
				setState(194);
				match(T__9);
				}
				}
				break;
			case CIRRUS:
				{
				{
				setState(195);
				match(CIRRUS);
				}
				}
				break;
			case CIRROCUMULUS:
				{
				{
				setState(196);
				match(CIRROCUMULUS);
				}
				}
				break;
			case ALTOCUMULUS:
				{
				{
				setState(197);
				match(ALTOCUMULUS);
				}
				}
				break;
			case STRATOCUMULUS:
				{
				{
				setState(198);
				match(STRATOCUMULUS);
				}
				}
				break;
			case T__10:
				{
				{
				setState(199);
				match(T__10);
				}
				}
				break;
			case T__11:
				{
				{
				setState(200);
				match(T__11);
				}
				}
				break;
			case T__12:
				{
				{
				setState(201);
				match(T__12);
				}
				}
				break;
			case CUMULONIMBUS:
				{
				{
				setState(202);
				match(CUMULONIMBUS);
				}
				}
				break;
			case NONE:
				{
				{
				setState(203);
				match(NONE);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
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

	public static class Sight_eofContext extends ParserRuleContext {
		public weather._ast.ASTSight ret =  null;
		public SightContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public SightContext sight() {
			return getRuleContext(SightContext.class,0);
		}
		public Sight_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sight_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterSight_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitSight_eof(this);
		}
	}

	public final Sight_eofContext sight_eof() throws RecognitionException {
		Sight_eofContext _localctx = new Sight_eofContext(_ctx, getState());
		enterRule(_localctx, 52, RULE_sight_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(206);
			((Sight_eofContext)_localctx).tmp = sight();
			((Sight_eofContext)_localctx).ret =  ((Sight_eofContext)_localctx).tmp.ret;
			setState(208);
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

	public static class SightContext extends ParserRuleContext {
		public weather._ast.ASTSight ret =  null;
		public UnitNumberContext tmp0;
		public TerminalNode SIGHT() { return getToken(WeatherAntlrParser.SIGHT, 0); }
		public UnitNumberContext unitNumber() {
			return getRuleContext(UnitNumberContext.class,0);
		}
		public TerminalNode UNLIMITED() { return getToken(WeatherAntlrParser.UNLIMITED, 0); }
		public SightContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_sight; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterSight(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitSight(this);
		}
	}

	public final SightContext sight() throws RecognitionException {
		SightContext _localctx = new SightContext(_ctx, getState());
		enterRule(_localctx, 54, RULE_sight);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTSight _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTSight();
		((SightContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			setState(210);
			match(SIGHT);
			}
			setState(215);
			switch (_input.LA(1)) {
			case TUnitNumber:
				{
				setState(211);
				((SightContext)_localctx).tmp0 = unitNumber();
				_aNode.setWeatherSight(_localctx.tmp0.ret);
				}
				break;
			case UNLIMITED:
				{
				{
				setState(214);
				match(UNLIMITED);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
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

	public static class WeatherPhenomena_eofContext extends ParserRuleContext {
		public weather._ast.ASTWeatherPhenomena ret =  null;
		public WeatherPhenomenaContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public WeatherPhenomenaContext weatherPhenomena() {
			return getRuleContext(WeatherPhenomenaContext.class,0);
		}
		public WeatherPhenomena_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_weatherPhenomena_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWeatherPhenomena_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWeatherPhenomena_eof(this);
		}
	}

	public final WeatherPhenomena_eofContext weatherPhenomena_eof() throws RecognitionException {
		WeatherPhenomena_eofContext _localctx = new WeatherPhenomena_eofContext(_ctx, getState());
		enterRule(_localctx, 56, RULE_weatherPhenomena_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(217);
			((WeatherPhenomena_eofContext)_localctx).tmp = weatherPhenomena();
			((WeatherPhenomena_eofContext)_localctx).ret =  ((WeatherPhenomena_eofContext)_localctx).tmp.ret;
			setState(219);
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

	public static class WeatherPhenomenaContext extends ParserRuleContext {
		public weather._ast.ASTWeatherPhenomena ret =  null;
		public Token tmp0;
		public Token tmp1;
		public TerminalNode FOG() { return getToken(WeatherAntlrParser.FOG, 0); }
		public TerminalNode LANDSPOUT() { return getToken(WeatherAntlrParser.LANDSPOUT, 0); }
		public TerminalNode WATERSPOUT() { return getToken(WeatherAntlrParser.WATERSPOUT, 0); }
		public TerminalNode GUSTNADO() { return getToken(WeatherAntlrParser.GUSTNADO, 0); }
		public TerminalNode THUNDERSTORM() { return getToken(WeatherAntlrParser.THUNDERSTORM, 0); }
		public TerminalNode LPAREN() { return getToken(WeatherAntlrParser.LPAREN, 0); }
		public TerminalNode RPAREN() { return getToken(WeatherAntlrParser.RPAREN, 0); }
		public List<TerminalNode> Numb() { return getTokens(WeatherAntlrParser.Numb); }
		public TerminalNode Numb(int i) {
			return getToken(WeatherAntlrParser.Numb, i);
		}
		public WeatherPhenomenaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_weatherPhenomena; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterWeatherPhenomena(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitWeatherPhenomena(this);
		}
	}

	public final WeatherPhenomenaContext weatherPhenomena() throws RecognitionException {
		WeatherPhenomenaContext _localctx = new WeatherPhenomenaContext(_ctx, getState());
		enterRule(_localctx, 58, RULE_weatherPhenomena);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTWeatherPhenomena _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTWeatherPhenomena();
		((WeatherPhenomenaContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		int _la;
		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			{
			setState(221);
			match(T__13);
			}
			setState(233);
			switch (_input.LA(1)) {
			case FOG:
				{
				{
				setState(222);
				match(FOG);
				}
				}
				break;
			case T__14:
				{
				{
				setState(223);
				match(T__14);
				}
				}
				break;
			case T__15:
				{
				{
				setState(224);
				match(T__15);
				}
				}
				break;
			case T__16:
				{
				{
				setState(225);
				match(T__16);
				}
				}
				break;
			case T__17:
				{
				{
				setState(226);
				match(T__17);
				}
				}
				break;
			case LANDSPOUT:
				{
				{
				setState(227);
				match(LANDSPOUT);
				}
				}
				break;
			case WATERSPOUT:
				{
				{
				setState(228);
				match(WATERSPOUT);
				}
				}
				break;
			case GUSTNADO:
				{
				{
				setState(229);
				match(GUSTNADO);
				}
				}
				break;
			case T__18:
				{
				{
				setState(230);
				match(T__18);
				}
				}
				break;
			case T__19:
				{
				{
				setState(231);
				match(T__19);
				}
				}
				break;
			case THUNDERSTORM:
				{
				{
				setState(232);
				match(THUNDERSTORM);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
			setState(243);
			_la = _input.LA(1);
			if (_la==LPAREN) {
				{
				{
				setState(235);
				match(LPAREN);
				}
				{
				{
				setState(236);
				((WeatherPhenomenaContext)_localctx).tmp0 = match(Numb);
				_aNode.setPosX(convertNumb(((WeatherPhenomenaContext)_localctx).tmp0));
				}
				}
				{
				{
				setState(239);
				((WeatherPhenomenaContext)_localctx).tmp1 = match(Numb);
				_aNode.setPosY(convertNumb(((WeatherPhenomenaContext)_localctx).tmp1));
				}
				}
				{
				setState(242);
				match(RPAREN);
				}
				}
			}

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

	public static class OpticalPhenomena_eofContext extends ParserRuleContext {
		public weather._ast.ASTOpticalPhenomena ret =  null;
		public OpticalPhenomenaContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public OpticalPhenomenaContext opticalPhenomena() {
			return getRuleContext(OpticalPhenomenaContext.class,0);
		}
		public OpticalPhenomena_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_opticalPhenomena_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterOpticalPhenomena_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitOpticalPhenomena_eof(this);
		}
	}

	public final OpticalPhenomena_eofContext opticalPhenomena_eof() throws RecognitionException {
		OpticalPhenomena_eofContext _localctx = new OpticalPhenomena_eofContext(_ctx, getState());
		enterRule(_localctx, 60, RULE_opticalPhenomena_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(245);
			((OpticalPhenomena_eofContext)_localctx).tmp = opticalPhenomena();
			((OpticalPhenomena_eofContext)_localctx).ret =  ((OpticalPhenomena_eofContext)_localctx).tmp.ret;
			setState(247);
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

	public static class OpticalPhenomenaContext extends ParserRuleContext {
		public weather._ast.ASTOpticalPhenomena ret =  null;
		public TerminalNode RAINBOW() { return getToken(WeatherAntlrParser.RAINBOW, 0); }
		public TerminalNode MIRAGE() { return getToken(WeatherAntlrParser.MIRAGE, 0); }
		public OpticalPhenomenaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_opticalPhenomena; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterOpticalPhenomena(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitOpticalPhenomena(this);
		}
	}

	public final OpticalPhenomenaContext opticalPhenomena() throws RecognitionException {
		OpticalPhenomenaContext _localctx = new OpticalPhenomenaContext(_ctx, getState());
		enterRule(_localctx, 62, RULE_opticalPhenomena);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTOpticalPhenomena _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTOpticalPhenomena();
		((OpticalPhenomenaContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			{
			setState(249);
			match(T__20);
			}
			setState(257);
			switch (_input.LA(1)) {
			case RAINBOW:
				{
				{
				setState(250);
				match(RAINBOW);
				}
				}
				break;
			case T__21:
				{
				{
				setState(251);
				match(T__21);
				}
				}
				break;
			case T__22:
				{
				{
				setState(252);
				match(T__22);
				}
				}
				break;
			case T__23:
				{
				{
				setState(253);
				match(T__23);
				}
				}
				break;
			case T__24:
				{
				{
				setState(254);
				match(T__24);
				}
				}
				break;
			case MIRAGE:
				{
				{
				setState(255);
				match(MIRAGE);
				}
				}
				break;
			case T__25:
				{
				{
				setState(256);
				match(T__25);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
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

	public static class ArtificialPhenomena_eofContext extends ParserRuleContext {
		public weather._ast.ASTArtificialPhenomena ret =  null;
		public ArtificialPhenomenaContext tmp;
		public TerminalNode EOF() { return getToken(WeatherAntlrParser.EOF, 0); }
		public ArtificialPhenomenaContext artificialPhenomena() {
			return getRuleContext(ArtificialPhenomenaContext.class,0);
		}
		public ArtificialPhenomena_eofContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_artificialPhenomena_eof; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterArtificialPhenomena_eof(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitArtificialPhenomena_eof(this);
		}
	}

	public final ArtificialPhenomena_eofContext artificialPhenomena_eof() throws RecognitionException {
		ArtificialPhenomena_eofContext _localctx = new ArtificialPhenomena_eofContext(_ctx, getState());
		enterRule(_localctx, 64, RULE_artificialPhenomena_eof);
		try {
			enterOuterAlt(_localctx, 1);
			{
			setState(259);
			((ArtificialPhenomena_eofContext)_localctx).tmp = artificialPhenomena();
			((ArtificialPhenomena_eofContext)_localctx).ret =  ((ArtificialPhenomena_eofContext)_localctx).tmp.ret;
			setState(261);
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

	public static class ArtificialPhenomenaContext extends ParserRuleContext {
		public weather._ast.ASTArtificialPhenomena ret =  null;
		public TerminalNode CONTRAILS() { return getToken(WeatherAntlrParser.CONTRAILS, 0); }
		public TerminalNode SMOG() { return getToken(WeatherAntlrParser.SMOG, 0); }
		public ArtificialPhenomenaContext(ParserRuleContext parent, int invokingState) {
			super(parent, invokingState);
		}
		@Override public int getRuleIndex() { return RULE_artificialPhenomena; }
		@Override
		public void enterRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).enterArtificialPhenomena(this);
		}
		@Override
		public void exitRule(ParseTreeListener listener) {
			if ( listener instanceof WeatherAntlrListener ) ((WeatherAntlrListener)listener).exitArtificialPhenomena(this);
		}
	}

	public final ArtificialPhenomenaContext artificialPhenomena() throws RecognitionException {
		ArtificialPhenomenaContext _localctx = new ArtificialPhenomenaContext(_ctx, getState());
		enterRule(_localctx, 66, RULE_artificialPhenomena);
		// ret is normally returned, a is used to be compatible with rule using the return construct
		weather._ast.ASTArtificialPhenomena _aNode = null;
		_aNode=weather._ast.WeatherNodeFactory.createASTArtificialPhenomena();
		((ArtificialPhenomenaContext)_localctx).ret = _aNode;
		_aNode.set_SourcePositionStart( computeStartPosition(_input.LT(1)));
		setActiveASTNode(_aNode);

		try {
			enterOuterAlt(_localctx, 1);
			{
			{
			{
			setState(263);
			match(T__26);
			}
			setState(267);
			switch (_input.LA(1)) {
			case CONTRAILS:
				{
				{
				setState(264);
				match(CONTRAILS);
				}
				}
				break;
			case SMOG:
				{
				{
				setState(265);
				match(SMOG);
				}
				}
				break;
			case T__27:
				{
				{
				setState(266);
				match(T__27);
				}
				}
				break;
			default:
				throw new NoViableAltException(this);
			}
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
		"\3\u0430\ud6d1\u8206\uad2d\u4417\uaef1\u8d80\uaadd\3M\u0110\4\2\t\2\4"+
		"\3\t\3\4\4\t\4\4\5\t\5\4\6\t\6\4\7\t\7\4\b\t\b\4\t\t\t\4\n\t\n\4\13\t"+
		"\13\4\f\t\f\4\r\t\r\4\16\t\16\4\17\t\17\4\20\t\20\4\21\t\21\4\22\t\22"+
		"\4\23\t\23\4\24\t\24\4\25\t\25\4\26\t\26\4\27\t\27\4\30\t\30\4\31\t\31"+
		"\4\32\t\32\4\33\t\33\4\34\t\34\4\35\t\35\4\36\t\36\4\37\t\37\4 \t \4!"+
		"\t!\4\"\t\"\4#\t#\3\2\3\2\3\2\3\2\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3\3"+
		"\3\3\3\3\3\3\5\3W\n\3\3\4\3\4\3\4\3\4\3\5\3\5\3\5\3\6\3\6\3\6\3\6\3\7"+
		"\3\7\3\7\3\b\3\b\3\b\3\b\3\t\3\t\3\t\3\n\3\n\3\n\3\n\3\13\3\13\3\13\3"+
		"\f\3\f\3\f\3\f\3\r\3\r\3\r\3\r\3\16\3\16\3\16\3\16\3\17\3\17\3\17\3\17"+
		"\3\20\3\20\3\20\3\20\3\21\3\21\3\21\3\21\3\22\3\22\3\22\3\22\3\23\3\23"+
		"\3\23\3\23\3\24\3\24\3\24\3\24\3\25\3\25\3\25\3\25\3\26\3\26\3\26\3\26"+
		"\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27\3\27"+
		"\3\27\3\27\5\27\u00b1\n\27\3\30\3\30\3\30\3\30\3\31\3\31\3\31\3\31\3\32"+
		"\3\32\3\32\3\32\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33\3\33"+
		"\3\33\3\33\3\33\3\33\3\33\5\33\u00cf\n\33\3\34\3\34\3\34\3\34\3\35\3\35"+
		"\3\35\3\35\3\35\5\35\u00da\n\35\3\36\3\36\3\36\3\36\3\37\3\37\3\37\3\37"+
		"\3\37\3\37\3\37\3\37\3\37\3\37\3\37\3\37\5\37\u00ec\n\37\3\37\3\37\3\37"+
		"\3\37\3\37\3\37\3\37\3\37\5\37\u00f6\n\37\3 \3 \3 \3 \3!\3!\3!\3!\3!\3"+
		"!\3!\3!\5!\u0104\n!\3\"\3\"\3\"\3\"\3#\3#\3#\3#\5#\u010e\n#\3#\2\2$\2"+
		"\4\6\b\n\f\16\20\22\24\26\30\32\34\36 \"$&(*,.\60\62\64\668:<>@BD\2\2"+
		"\u011f\2F\3\2\2\2\4V\3\2\2\2\6X\3\2\2\2\b\\\3\2\2\2\n_\3\2\2\2\fc\3\2"+
		"\2\2\16f\3\2\2\2\20j\3\2\2\2\22m\3\2\2\2\24q\3\2\2\2\26t\3\2\2\2\30x\3"+
		"\2\2\2\32|\3\2\2\2\34\u0080\3\2\2\2\36\u0084\3\2\2\2 \u0088\3\2\2\2\""+
		"\u008c\3\2\2\2$\u0090\3\2\2\2&\u0094\3\2\2\2(\u0098\3\2\2\2*\u009c\3\2"+
		"\2\2,\u00a0\3\2\2\2.\u00b2\3\2\2\2\60\u00b6\3\2\2\2\62\u00ba\3\2\2\2\64"+
		"\u00be\3\2\2\2\66\u00d0\3\2\2\28\u00d4\3\2\2\2:\u00db\3\2\2\2<\u00df\3"+
		"\2\2\2>\u00f7\3\2\2\2@\u00fb\3\2\2\2B\u0105\3\2\2\2D\u0109\3\2\2\2FG\5"+
		"\4\3\2GH\b\2\1\2HI\7\2\2\3I\3\3\2\2\2JK\5\b\5\2KL\b\3\1\2LW\3\2\2\2MN"+
		"\5\f\7\2NO\b\3\1\2OW\3\2\2\2PQ\5\24\13\2QR\b\3\1\2RW\3\2\2\2ST\5\20\t"+
		"\2TU\b\3\1\2UW\3\2\2\2VJ\3\2\2\2VM\3\2\2\2VP\3\2\2\2VS\3\2\2\2W\5\3\2"+
		"\2\2XY\5\b\5\2YZ\b\4\1\2Z[\7\2\2\3[\7\3\2\2\2\\]\7E\2\2]^\b\5\1\2^\t\3"+
		"\2\2\2_`\5\f\7\2`a\b\6\1\2ab\7\2\2\3b\13\3\2\2\2cd\7F\2\2de\b\7\1\2e\r"+
		"\3\2\2\2fg\5\20\t\2gh\b\b\1\2hi\7\2\2\3i\17\3\2\2\2jk\7G\2\2kl\b\t\1\2"+
		"l\21\3\2\2\2mn\5\24\13\2no\b\n\1\2op\7\2\2\3p\23\3\2\2\2qr\7I\2\2rs\b"+
		"\13\1\2s\25\3\2\2\2tu\5\30\r\2uv\b\f\1\2vw\7\2\2\3w\27\3\2\2\2xy\7*\2"+
		"\2yz\5\20\t\2z{\b\r\1\2{\31\3\2\2\2|}\5\34\17\2}~\b\16\1\2~\177\7\2\2"+
		"\3\177\33\3\2\2\2\u0080\u0081\7,\2\2\u0081\u0082\5\20\t\2\u0082\u0083"+
		"\b\17\1\2\u0083\35\3\2\2\2\u0084\u0085\5 \21\2\u0085\u0086\b\20\1\2\u0086"+
		"\u0087\7\2\2\3\u0087\37\3\2\2\2\u0088\u0089\78\2\2\u0089\u008a\5\20\t"+
		"\2\u008a\u008b\b\21\1\2\u008b!\3\2\2\2\u008c\u008d\5$\23\2\u008d\u008e"+
		"\b\22\1\2\u008e\u008f\7\2\2\3\u008f#\3\2\2\2\u0090\u0091\7@\2\2\u0091"+
		"\u0092\5\20\t\2\u0092\u0093\b\23\1\2\u0093%\3\2\2\2\u0094\u0095\5(\25"+
		"\2\u0095\u0096\b\24\1\2\u0096\u0097\7\2\2\3\u0097\'\3\2\2\2\u0098\u0099"+
		"\7;\2\2\u0099\u009a\5\20\t\2\u009a\u009b\b\25\1\2\u009b)\3\2\2\2\u009c"+
		"\u009d\5,\27\2\u009d\u009e\b\26\1\2\u009e\u009f\7\2\2\3\u009f+\3\2\2\2"+
		"\u00a0\u00b0\7\3\2\2\u00a1\u00b1\7$\2\2\u00a2\u00b1\79\2\2\u00a3\u00b1"+
		"\7\61\2\2\u00a4\u00b1\7\4\2\2\u00a5\u00b1\7\5\2\2\u00a6\u00b1\7\6\2\2"+
		"\u00a7\u00b1\7 \2\2\u00a8\u00b1\7?\2\2\u00a9\u00b1\7\7\2\2\u00aa\u00b1"+
		"\7\b\2\2\u00ab\u00b1\7.\2\2\u00ac\u00b1\7:\2\2\u00ad\u00b1\7\t\2\2\u00ae"+
		"\u00b1\7\60\2\2\u00af\u00b1\7\n\2\2\u00b0\u00a1\3\2\2\2\u00b0\u00a2\3"+
		"\2\2\2\u00b0\u00a3\3\2\2\2\u00b0\u00a4\3\2\2\2\u00b0\u00a5\3\2\2\2\u00b0"+
		"\u00a6\3\2\2\2\u00b0\u00a7\3\2\2\2\u00b0\u00a8\3\2\2\2\u00b0\u00a9\3\2"+
		"\2\2\u00b0\u00aa\3\2\2\2\u00b0\u00ab\3\2\2\2\u00b0\u00ac\3\2\2\2\u00b0"+
		"\u00ad\3\2\2\2\u00b0\u00ae\3\2\2\2\u00b0\u00af\3\2\2\2\u00b1-\3\2\2\2"+
		"\u00b2\u00b3\5\60\31\2\u00b3\u00b4\b\30\1\2\u00b4\u00b5\7\2\2\3\u00b5"+
		"/\3\2\2\2\u00b6\u00b7\7\13\2\2\u00b7\u00b8\5\20\t\2\u00b8\u00b9\b\31\1"+
		"\2\u00b9\61\3\2\2\2\u00ba\u00bb\5\64\33\2\u00bb\u00bc\b\32\1\2\u00bc\u00bd"+
		"\7\2\2\3\u00bd\63\3\2\2\2\u00be\u00ce\7>\2\2\u00bf\u00cf\7A\2\2\u00c0"+
		"\u00c1\7!\2\2\u00c1\u00cf\7\'\2\2\u00c2\u00cf\7%\2\2\u00c3\u00cf\7/\2"+
		"\2\u00c4\u00cf\7\f\2\2\u00c5\u00cf\7\64\2\2\u00c6\u00cf\7<\2\2\u00c7\u00cf"+
		"\7\62\2\2\u00c8\u00cf\7(\2\2\u00c9\u00cf\7\r\2\2\u00ca\u00cf\7\16\2\2"+
		"\u00cb\u00cf\7\17\2\2\u00cc\u00cf\7&\2\2\u00cd\u00cf\7$\2\2\u00ce\u00bf"+
		"\3\2\2\2\u00ce\u00c0\3\2\2\2\u00ce\u00c2\3\2\2\2\u00ce\u00c3\3\2\2\2\u00ce"+
		"\u00c4\3\2\2\2\u00ce\u00c5\3\2\2\2\u00ce\u00c6\3\2\2\2\u00ce\u00c7\3\2"+
		"\2\2\u00ce\u00c8\3\2\2\2\u00ce\u00c9\3\2\2\2\u00ce\u00ca\3\2\2\2\u00ce"+
		"\u00cb\3\2\2\2\u00ce\u00cc\3\2\2\2\u00ce\u00cd\3\2\2\2\u00cf\65\3\2\2"+
		"\2\u00d0\u00d1\58\35\2\u00d1\u00d2\b\34\1\2\u00d2\u00d3\7\2\2\3\u00d3"+
		"\67\3\2\2\2\u00d4\u00d9\7+\2\2\u00d5\u00d6\5\20\t\2\u00d6\u00d7\b\35\1"+
		"\2\u00d7\u00da\3\2\2\2\u00d8\u00da\7\63\2\2\u00d9\u00d5\3\2\2\2\u00d9"+
		"\u00d8\3\2\2\2\u00da9\3\2\2\2\u00db\u00dc\5<\37\2\u00dc\u00dd\b\36\1\2"+
		"\u00dd\u00de\7\2\2\3\u00de;\3\2\2\2\u00df\u00eb\7\20\2\2\u00e0\u00ec\7"+
		"C\2\2\u00e1\u00ec\7\21\2\2\u00e2\u00ec\7\22\2\2\u00e3\u00ec\7\23\2\2\u00e4"+
		"\u00ec\7\24\2\2\u00e5\u00ec\7#\2\2\u00e6\u00ec\7-\2\2\u00e7\u00ec\7\65"+
		"\2\2\u00e8\u00ec\7\25\2\2\u00e9\u00ec\7\26\2\2\u00ea\u00ec\7B\2\2\u00eb"+
		"\u00e0\3\2\2\2\u00eb\u00e1\3\2\2\2\u00eb\u00e2\3\2\2\2\u00eb\u00e3\3\2"+
		"\2\2\u00eb\u00e4\3\2\2\2\u00eb\u00e5\3\2\2\2\u00eb\u00e6\3\2\2\2\u00eb"+
		"\u00e7\3\2\2\2\u00eb\u00e8\3\2\2\2\u00eb\u00e9\3\2\2\2\u00eb\u00ea\3\2"+
		"\2\2\u00ec\u00f5\3\2\2\2\u00ed\u00ee\7\66\2\2\u00ee\u00ef\7D\2\2\u00ef"+
		"\u00f0\b\37\1\2\u00f0\u00f1\3\2\2\2\u00f1\u00f2\7D\2\2\u00f2\u00f3\b\37"+
		"\1\2\u00f3\u00f4\3\2\2\2\u00f4\u00f6\7\67\2\2\u00f5\u00ed\3\2\2\2\u00f5"+
		"\u00f6\3\2\2\2\u00f6=\3\2\2\2\u00f7\u00f8\5@!\2\u00f8\u00f9\b \1\2\u00f9"+
		"\u00fa\7\2\2\3\u00fa?\3\2\2\2\u00fb\u0103\7\27\2\2\u00fc\u0104\7=\2\2"+
		"\u00fd\u0104\7\30\2\2\u00fe\u0104\7\31\2\2\u00ff\u0104\7\32\2\2\u0100"+
		"\u0104\7\33\2\2\u0101\u0104\7)\2\2\u0102\u0104\7\34\2\2\u0103\u00fc\3"+
		"\2\2\2\u0103\u00fd\3\2\2\2\u0103\u00fe\3\2\2\2\u0103\u00ff\3\2\2\2\u0103"+
		"\u0100\3\2\2\2\u0103\u0101\3\2\2\2\u0103\u0102\3\2\2\2\u0104A\3\2\2\2"+
		"\u0105\u0106\5D#\2\u0106\u0107\b\"\1\2\u0107\u0108\7\2\2\3\u0108C\3\2"+
		"\2\2\u0109\u010d\7\35\2\2\u010a\u010e\7\37\2\2\u010b\u010e\7\"\2\2\u010c"+
		"\u010e\7\36\2\2\u010d\u010a\3\2\2\2\u010d\u010b\3\2\2\2\u010d\u010c\3"+
		"\2\2\2\u010eE\3\2\2\2\nV\u00b0\u00ce\u00d9\u00eb\u00f5\u0103\u010d";
	public static final ATN _ATN =
		new ATNDeserializer().deserialize(_serializedATN.toCharArray());
	static {
		_decisionToDFA = new DFA[_ATN.getNumberOfDecisions()];
		for (int i = 0; i < _ATN.getNumberOfDecisions(); i++) {
			_decisionToDFA[i] = new DFA(_ATN.getDecisionState(i), i);
		}
	}
}