#include "configReader.h"

const char* synthesisMethodName(SynthesisMethod method) {
	switch (method) {
	case SynthesisMethod::CDC: return "cdc";
	case SynthesisMethod::AUTOMATICA_SCAN: return "automatica_scan";
	case SynthesisMethod::AUTOMATICA_THRESHOLD: return "automatica_threshold";
	case SynthesisMethod::THRESHOLD: return "threshold";
	case SynthesisMethod::BITMAP_REFERENCE: return "bitmap_reference";
	case SynthesisMethod::THRESHOLD_CPU_REFERENCE: return "threshold_cpu_reference";
	}
	return "invalid";
}

const char* transitionBackendName(TransitionBackend backend) {
	return backend == TransitionBackend::INLINE ? "inline" : "precomputed";
}

const char* boundarySemanticsName(BoundarySemantics semantics) {
	return semantics == BoundarySemantics::FAVORABLE_SATURATING
		? "favorable_saturating" : "strict_unsafe";
}

//--------
// Define the singleton object + the class of the defaults
//--------
defaultConfiguration defaultConfiguration::s_singleton;
defaultConfiguration::defaultConfiguration()
{
	m_schema[0] = "data = string";
	m_schema[1] = "growth = scope";
	m_schema[2] = "growth.codeonly = boolean";
	m_schema[3] = "growth.finishcode_1 = string";
	m_schema[4] = "growth.finishcode_10 = string";
	m_schema[5] = "growth.finishcode_11 = string";
	m_schema[6] = "growth.finishcode_12 = string";
	m_schema[7] = "growth.finishcode_13 = string";
	m_schema[8] = "growth.finishcode_14 = string";
	m_schema[9] = "growth.finishcode_15 = string";
	m_schema[10] = "growth.finishcode_16 = string";
	m_schema[11] = "growth.finishcode_17 = string";
	m_schema[12] = "growth.finishcode_18 = string";
	m_schema[13] = "growth.finishcode_19 = string";
	m_schema[14] = "growth.finishcode_2 = string";
	m_schema[15] = "growth.finishcode_20 = string";
	m_schema[16] = "growth.finishcode_21 = string";
	m_schema[17] = "growth.finishcode_22 = string";
	m_schema[18] = "growth.finishcode_23 = string";
	m_schema[19] = "growth.finishcode_24 = string";
	m_schema[20] = "growth.finishcode_25 = string";
	m_schema[21] = "growth.finishcode_26 = string";
	m_schema[22] = "growth.finishcode_27 = string";
	m_schema[23] = "growth.finishcode_28 = string";
	m_schema[24] = "growth.finishcode_29 = string";
	m_schema[25] = "growth.finishcode_3 = string";
	m_schema[26] = "growth.finishcode_30 = string";
	m_schema[27] = "growth.finishcode_4 = string";
	m_schema[28] = "growth.finishcode_5 = string";
	m_schema[29] = "growth.finishcode_6 = string";
	m_schema[30] = "growth.finishcode_7 = string";
	m_schema[31] = "growth.finishcode_8 = string";
	m_schema[32] = "growth.finishcode_9 = string";
	m_schema[33] = "growth.initcode_1 = string";
	m_schema[34] = "growth.initcode_10 = string";
	m_schema[35] = "growth.initcode_11 = string";
	m_schema[36] = "growth.initcode_12 = string";
	m_schema[37] = "growth.initcode_13 = string";
	m_schema[38] = "growth.initcode_14 = string";
	m_schema[39] = "growth.initcode_15 = string";
	m_schema[40] = "growth.initcode_16 = string";
	m_schema[41] = "growth.initcode_17 = string";
	m_schema[42] = "growth.initcode_18 = string";
	m_schema[43] = "growth.initcode_19 = string";
	m_schema[44] = "growth.initcode_2 = string";
	m_schema[45] = "growth.initcode_20 = string";
	m_schema[46] = "growth.initcode_21 = string";
	m_schema[47] = "growth.initcode_22 = string";
	m_schema[48] = "growth.initcode_23 = string";
	m_schema[49] = "growth.initcode_24 = string";
	m_schema[50] = "growth.initcode_25 = string";
	m_schema[51] = "growth.initcode_26 = string";
	m_schema[52] = "growth.initcode_27 = string";
	m_schema[53] = "growth.initcode_28 = string";
	m_schema[54] = "growth.initcode_29 = string";
	m_schema[55] = "growth.initcode_3 = string";
	m_schema[56] = "growth.initcode_30 = string";
	m_schema[57] = "growth.initcode_4 = string";
	m_schema[58] = "growth.initcode_5 = string";
	m_schema[59] = "growth.initcode_6 = string";
	m_schema[60] = "growth.initcode_7 = string";
	m_schema[61] = "growth.initcode_8 = string";
	m_schema[62] = "growth.initcode_9 = string";
	m_schema[63] = "growth.maxposts = int";
	m_schema[64] = "growth.rr1 = string";
	m_schema[65] = "growth.rr10 = string";
	m_schema[66] = "growth.rr100 = string";
	m_schema[67] = "growth.rr11 = string";
	m_schema[68] = "growth.rr12 = string";
	m_schema[69] = "growth.rr13 = string";
	m_schema[70] = "growth.rr14 = string";
	m_schema[71] = "growth.rr15 = string";
	m_schema[72] = "growth.rr16 = string";
	m_schema[73] = "growth.rr17 = string";
	m_schema[74] = "growth.rr18 = string";
	m_schema[75] = "growth.rr19 = string";
	m_schema[76] = "growth.rr2 = string";
	m_schema[77] = "growth.rr20 = string";
	m_schema[78] = "growth.rr21 = string";
	m_schema[79] = "growth.rr22 = string";
	m_schema[80] = "growth.rr23 = string";
	m_schema[81] = "growth.rr24 = string";
	m_schema[82] = "growth.rr25 = string";
	m_schema[83] = "growth.rr26 = string";
	m_schema[84] = "growth.rr27 = string";
	m_schema[85] = "growth.rr28 = string";
	m_schema[86] = "growth.rr29 = string";
	m_schema[87] = "growth.rr3 = string";
	m_schema[88] = "growth.rr30 = string";
	m_schema[89] = "growth.rr31 = string";
	m_schema[90] = "growth.rr32 = string";
	m_schema[91] = "growth.rr33 = string";
	m_schema[92] = "growth.rr34 = string";
	m_schema[93] = "growth.rr35 = string";
	m_schema[94] = "growth.rr36 = string";
	m_schema[95] = "growth.rr37 = string";
	m_schema[96] = "growth.rr38 = string";
	m_schema[97] = "growth.rr39 = string";
	m_schema[98] = "growth.rr4 = string";
	m_schema[99] = "growth.rr40 = string";
	m_schema[100] = "growth.rr41 = string";
	m_schema[101] = "growth.rr42 = string";
	m_schema[102] = "growth.rr43 = string";
	m_schema[103] = "growth.rr44 = string";
	m_schema[104] = "growth.rr45 = string";
	m_schema[105] = "growth.rr46 = string";
	m_schema[106] = "growth.rr47 = string";
	m_schema[107] = "growth.rr48 = string";
	m_schema[108] = "growth.rr49 = string";
	m_schema[109] = "growth.rr5 = string";
	m_schema[110] = "growth.rr50 = string";
	m_schema[111] = "growth.rr51 = string";
	m_schema[112] = "growth.rr52 = string";
	m_schema[113] = "growth.rr53 = string";
	m_schema[114] = "growth.rr54 = string";
	m_schema[115] = "growth.rr55 = string";
	m_schema[116] = "growth.rr56 = string";
	m_schema[117] = "growth.rr57 = string";
	m_schema[118] = "growth.rr58 = string";
	m_schema[119] = "growth.rr59 = string";
	m_schema[120] = "growth.rr6 = string";
	m_schema[121] = "growth.rr60 = string";
	m_schema[122] = "growth.rr61 = string";
	m_schema[123] = "growth.rr62 = string";
	m_schema[124] = "growth.rr63 = string";
	m_schema[125] = "growth.rr64 = string";
	m_schema[126] = "growth.rr65 = string";
	m_schema[127] = "growth.rr66 = string";
	m_schema[128] = "growth.rr67 = string";
	m_schema[129] = "growth.rr68 = string";
	m_schema[130] = "growth.rr69 = string";
	m_schema[131] = "growth.rr7 = string";
	m_schema[132] = "growth.rr70 = string";
	m_schema[133] = "growth.rr71 = string";
	m_schema[134] = "growth.rr72 = string";
	m_schema[135] = "growth.rr73 = string";
	m_schema[136] = "growth.rr74 = string";
	m_schema[137] = "growth.rr75 = string";
	m_schema[138] = "growth.rr76 = string";
	m_schema[139] = "growth.rr77 = string";
	m_schema[140] = "growth.rr78 = string";
	m_schema[141] = "growth.rr79 = string";
	m_schema[142] = "growth.rr8 = string";
	m_schema[143] = "growth.rr80 = string";
	m_schema[144] = "growth.rr81 = string";
	m_schema[145] = "growth.rr82 = string";
	m_schema[146] = "growth.rr83 = string";
	m_schema[147] = "growth.rr84 = string";
	m_schema[148] = "growth.rr85 = string";
	m_schema[149] = "growth.rr86 = string";
	m_schema[150] = "growth.rr87 = string";
	m_schema[151] = "growth.rr88 = string";
	m_schema[152] = "growth.rr89 = string";
	m_schema[153] = "growth.rr9 = string";
	m_schema[154] = "growth.rr90 = string";
	m_schema[155] = "growth.rr91 = string";
	m_schema[156] = "growth.rr92 = string";
	m_schema[157] = "growth.rr93 = string";
	m_schema[158] = "growth.rr94 = string";
	m_schema[159] = "growth.rr95 = string";
	m_schema[160] = "growth.rr96 = string";
	m_schema[161] = "growth.rr97 = string";
	m_schema[162] = "growth.rr98 = string";
	m_schema[163] = "growth.rr99 = string";
	m_schema[164] = "growth.useode = boolean";
	m_schema[165] = "implement = string";
	m_schema[166] = "inputs = scope";
	m_schema[167] = "inputs.dim = int";
	m_schema[168] = "inputs.err = string";
	m_schema[169] = "inputs.eta = string";
	m_schema[170] = "inputs.lb = string";
	m_schema[171] = "inputs.ub = string";
	m_schema[172] = "obstacles = scope";
	m_schema[173] = "obstacles.count = int";
	m_schema[174] = "obstacles.ob1 = scope";
	m_schema[175] = "obstacles.ob1.h = string";
	m_schema[176] = "obstacles.ob1.type = string";
	m_schema[177] = "obstacles.ob10 = scope";
	m_schema[178] = "obstacles.ob10.h = string";
	m_schema[179] = "obstacles.ob10.type = string";
	m_schema[180] = "obstacles.ob11 = scope";
	m_schema[181] = "obstacles.ob11.h = string";
	m_schema[182] = "obstacles.ob11.type = string";
	m_schema[183] = "obstacles.ob12 = scope";
	m_schema[184] = "obstacles.ob12.h = string";
	m_schema[185] = "obstacles.ob12.type = string";
	m_schema[186] = "obstacles.ob13 = scope";
	m_schema[187] = "obstacles.ob13.h = string";
	m_schema[188] = "obstacles.ob13.type = string";
	m_schema[189] = "obstacles.ob14 = scope";
	m_schema[190] = "obstacles.ob14.h = string";
	m_schema[191] = "obstacles.ob14.type = string";
	m_schema[192] = "obstacles.ob15 = scope";
	m_schema[193] = "obstacles.ob15.h = string";
	m_schema[194] = "obstacles.ob15.type = string";
	m_schema[195] = "obstacles.ob16 = scope";
	m_schema[196] = "obstacles.ob16.h = string";
	m_schema[197] = "obstacles.ob16.type = string";
	m_schema[198] = "obstacles.ob17 = scope";
	m_schema[199] = "obstacles.ob17.h = string";
	m_schema[200] = "obstacles.ob17.type = string";
	m_schema[201] = "obstacles.ob18 = scope";
	m_schema[202] = "obstacles.ob18.h = string";
	m_schema[203] = "obstacles.ob18.type = string";
	m_schema[204] = "obstacles.ob19 = scope";
	m_schema[205] = "obstacles.ob19.h = string";
	m_schema[206] = "obstacles.ob19.type = string";
	m_schema[207] = "obstacles.ob2 = scope";
	m_schema[208] = "obstacles.ob2.h = string";
	m_schema[209] = "obstacles.ob2.type = string";
	m_schema[210] = "obstacles.ob20 = scope";
	m_schema[211] = "obstacles.ob20.h = string";
	m_schema[212] = "obstacles.ob20.type = string";
	m_schema[213] = "obstacles.ob21 = scope";
	m_schema[214] = "obstacles.ob21.h = string";
	m_schema[215] = "obstacles.ob21.type = string";
	m_schema[216] = "obstacles.ob22 = scope";
	m_schema[217] = "obstacles.ob22.h = string";
	m_schema[218] = "obstacles.ob22.type = string";
	m_schema[219] = "obstacles.ob23 = scope";
	m_schema[220] = "obstacles.ob23.h = string";
	m_schema[221] = "obstacles.ob23.type = string";
	m_schema[222] = "obstacles.ob24 = scope";
	m_schema[223] = "obstacles.ob24.h = string";
	m_schema[224] = "obstacles.ob24.type = string";
	m_schema[225] = "obstacles.ob25 = scope";
	m_schema[226] = "obstacles.ob25.h = string";
	m_schema[227] = "obstacles.ob25.type = string";
	m_schema[228] = "obstacles.ob26 = scope";
	m_schema[229] = "obstacles.ob26.h = string";
	m_schema[230] = "obstacles.ob26.type = string";
	m_schema[231] = "obstacles.ob27 = scope";
	m_schema[232] = "obstacles.ob27.h = string";
	m_schema[233] = "obstacles.ob27.type = string";
	m_schema[234] = "obstacles.ob28 = scope";
	m_schema[235] = "obstacles.ob28.h = string";
	m_schema[236] = "obstacles.ob28.type = string";
	m_schema[237] = "obstacles.ob29 = scope";
	m_schema[238] = "obstacles.ob29.h = string";
	m_schema[239] = "obstacles.ob29.type = string";
	m_schema[240] = "obstacles.ob3 = scope";
	m_schema[241] = "obstacles.ob3.h = string";
	m_schema[242] = "obstacles.ob3.type = string";
	m_schema[243] = "obstacles.ob30 = scope";
	m_schema[244] = "obstacles.ob30.h = string";
	m_schema[245] = "obstacles.ob30.type = string";
	m_schema[246] = "obstacles.ob31 = scope";
	m_schema[247] = "obstacles.ob31.h = string";
	m_schema[248] = "obstacles.ob31.type = string";
	m_schema[249] = "obstacles.ob32 = scope";
	m_schema[250] = "obstacles.ob32.h = string";
	m_schema[251] = "obstacles.ob32.type = string";
	m_schema[252] = "obstacles.ob33 = scope";
	m_schema[253] = "obstacles.ob33.h = string";
	m_schema[254] = "obstacles.ob33.type = string";
	m_schema[255] = "obstacles.ob34 = scope";
	m_schema[256] = "obstacles.ob34.h = string";
	m_schema[257] = "obstacles.ob34.type = string";
	m_schema[258] = "obstacles.ob35 = scope";
	m_schema[259] = "obstacles.ob35.h = string";
	m_schema[260] = "obstacles.ob35.type = string";
	m_schema[261] = "obstacles.ob36 = scope";
	m_schema[262] = "obstacles.ob36.h = string";
	m_schema[263] = "obstacles.ob36.type = string";
	m_schema[264] = "obstacles.ob37 = scope";
	m_schema[265] = "obstacles.ob37.h = string";
	m_schema[266] = "obstacles.ob37.type = string";
	m_schema[267] = "obstacles.ob38 = scope";
	m_schema[268] = "obstacles.ob38.h = string";
	m_schema[269] = "obstacles.ob38.type = string";
	m_schema[270] = "obstacles.ob39 = scope";
	m_schema[271] = "obstacles.ob39.h = string";
	m_schema[272] = "obstacles.ob39.type = string";
	m_schema[273] = "obstacles.ob4 = scope";
	m_schema[274] = "obstacles.ob4.h = string";
	m_schema[275] = "obstacles.ob4.type = string";
	m_schema[276] = "obstacles.ob40 = scope";
	m_schema[277] = "obstacles.ob40.h = string";
	m_schema[278] = "obstacles.ob40.type = string";
	m_schema[279] = "obstacles.ob41 = scope";
	m_schema[280] = "obstacles.ob41.h = string";
	m_schema[281] = "obstacles.ob41.type = string";
	m_schema[282] = "obstacles.ob42 = scope";
	m_schema[283] = "obstacles.ob42.h = string";
	m_schema[284] = "obstacles.ob42.type = string";
	m_schema[285] = "obstacles.ob43 = scope";
	m_schema[286] = "obstacles.ob43.h = string";
	m_schema[287] = "obstacles.ob43.type = string";
	m_schema[288] = "obstacles.ob44 = scope";
	m_schema[289] = "obstacles.ob44.h = string";
	m_schema[290] = "obstacles.ob44.type = string";
	m_schema[291] = "obstacles.ob45 = scope";
	m_schema[292] = "obstacles.ob45.h = string";
	m_schema[293] = "obstacles.ob45.type = string";
	m_schema[294] = "obstacles.ob46 = scope";
	m_schema[295] = "obstacles.ob46.h = string";
	m_schema[296] = "obstacles.ob46.type = string";
	m_schema[297] = "obstacles.ob47 = scope";
	m_schema[298] = "obstacles.ob47.h = string";
	m_schema[299] = "obstacles.ob47.type = string";
	m_schema[300] = "obstacles.ob48 = scope";
	m_schema[301] = "obstacles.ob48.h = string";
	m_schema[302] = "obstacles.ob48.type = string";
	m_schema[303] = "obstacles.ob49 = scope";
	m_schema[304] = "obstacles.ob49.h = string";
	m_schema[305] = "obstacles.ob49.type = string";
	m_schema[306] = "obstacles.ob5 = scope";
	m_schema[307] = "obstacles.ob5.h = string";
	m_schema[308] = "obstacles.ob5.type = string";
	m_schema[309] = "obstacles.ob50 = scope";
	m_schema[310] = "obstacles.ob50.h = string";
	m_schema[311] = "obstacles.ob50.type = string";
	m_schema[312] = "obstacles.ob6 = scope";
	m_schema[313] = "obstacles.ob6.h = string";
	m_schema[314] = "obstacles.ob6.type = string";
	m_schema[315] = "obstacles.ob7 = scope";
	m_schema[316] = "obstacles.ob7.h = string";
	m_schema[317] = "obstacles.ob7.type = string";
	m_schema[318] = "obstacles.ob8 = scope";
	m_schema[319] = "obstacles.ob8.h = string";
	m_schema[320] = "obstacles.ob8.type = string";
	m_schema[321] = "obstacles.ob9 = scope";
	m_schema[322] = "obstacles.ob9.h = string";
	m_schema[323] = "obstacles.ob9.type = string";
	m_schema[324] = "online_mode = boolean";
	m_schema[325] = "post = scope";
	m_schema[326] = "post.codeonly = boolean";
	m_schema[327] = "post.finishcode_1 = string";
	m_schema[328] = "post.finishcode_10 = string";
	m_schema[329] = "post.finishcode_11 = string";
	m_schema[330] = "post.finishcode_12 = string";
	m_schema[331] = "post.finishcode_13 = string";
	m_schema[332] = "post.finishcode_14 = string";
	m_schema[333] = "post.finishcode_15 = string";
	m_schema[334] = "post.finishcode_16 = string";
	m_schema[335] = "post.finishcode_17 = string";
	m_schema[336] = "post.finishcode_18 = string";
	m_schema[337] = "post.finishcode_19 = string";
	m_schema[338] = "post.finishcode_2 = string";
	m_schema[339] = "post.finishcode_20 = string";
	m_schema[340] = "post.finishcode_21 = string";
	m_schema[341] = "post.finishcode_22 = string";
	m_schema[342] = "post.finishcode_23 = string";
	m_schema[343] = "post.finishcode_24 = string";
	m_schema[344] = "post.finishcode_25 = string";
	m_schema[345] = "post.finishcode_26 = string";
	m_schema[346] = "post.finishcode_27 = string";
	m_schema[347] = "post.finishcode_28 = string";
	m_schema[348] = "post.finishcode_29 = string";
	m_schema[349] = "post.finishcode_3 = string";
	m_schema[350] = "post.finishcode_30 = string";
	m_schema[351] = "post.finishcode_4 = string";
	m_schema[352] = "post.finishcode_5 = string";
	m_schema[353] = "post.finishcode_6 = string";
	m_schema[354] = "post.finishcode_7 = string";
	m_schema[355] = "post.finishcode_8 = string";
	m_schema[356] = "post.finishcode_9 = string";
	m_schema[357] = "post.initcode_1 = string";
	m_schema[358] = "post.initcode_10 = string";
	m_schema[359] = "post.initcode_11 = string";
	m_schema[360] = "post.initcode_12 = string";
	m_schema[361] = "post.initcode_13 = string";
	m_schema[362] = "post.initcode_14 = string";
	m_schema[363] = "post.initcode_15 = string";
	m_schema[364] = "post.initcode_16 = string";
	m_schema[365] = "post.initcode_17 = string";
	m_schema[366] = "post.initcode_18 = string";
	m_schema[367] = "post.initcode_19 = string";
	m_schema[368] = "post.initcode_2 = string";
	m_schema[369] = "post.initcode_20 = string";
	m_schema[370] = "post.initcode_21 = string";
	m_schema[371] = "post.initcode_22 = string";
	m_schema[372] = "post.initcode_23 = string";
	m_schema[373] = "post.initcode_24 = string";
	m_schema[374] = "post.initcode_25 = string";
	m_schema[375] = "post.initcode_26 = string";
	m_schema[376] = "post.initcode_27 = string";
	m_schema[377] = "post.initcode_28 = string";
	m_schema[378] = "post.initcode_29 = string";
	m_schema[379] = "post.initcode_3 = string";
	m_schema[380] = "post.initcode_30 = string";
	m_schema[381] = "post.initcode_4 = string";
	m_schema[382] = "post.initcode_5 = string";
	m_schema[383] = "post.initcode_6 = string";
	m_schema[384] = "post.initcode_7 = string";
	m_schema[385] = "post.initcode_8 = string";
	m_schema[386] = "post.initcode_9 = string";
	m_schema[387] = "post.useode = boolean";
	m_schema[388] = "post.xx1 = string";
	m_schema[389] = "post.xx10 = string";
	m_schema[390] = "post.xx100 = string";
	m_schema[391] = "post.xx11 = string";
	m_schema[392] = "post.xx12 = string";
	m_schema[393] = "post.xx13 = string";
	m_schema[394] = "post.xx14 = string";
	m_schema[395] = "post.xx15 = string";
	m_schema[396] = "post.xx16 = string";
	m_schema[397] = "post.xx17 = string";
	m_schema[398] = "post.xx18 = string";
	m_schema[399] = "post.xx19 = string";
	m_schema[400] = "post.xx2 = string";
	m_schema[401] = "post.xx20 = string";
	m_schema[402] = "post.xx21 = string";
	m_schema[403] = "post.xx22 = string";
	m_schema[404] = "post.xx23 = string";
	m_schema[405] = "post.xx24 = string";
	m_schema[406] = "post.xx25 = string";
	m_schema[407] = "post.xx26 = string";
	m_schema[408] = "post.xx27 = string";
	m_schema[409] = "post.xx28 = string";
	m_schema[410] = "post.xx29 = string";
	m_schema[411] = "post.xx3 = string";
	m_schema[412] = "post.xx30 = string";
	m_schema[413] = "post.xx31 = string";
	m_schema[414] = "post.xx32 = string";
	m_schema[415] = "post.xx33 = string";
	m_schema[416] = "post.xx34 = string";
	m_schema[417] = "post.xx35 = string";
	m_schema[418] = "post.xx36 = string";
	m_schema[419] = "post.xx37 = string";
	m_schema[420] = "post.xx38 = string";
	m_schema[421] = "post.xx39 = string";
	m_schema[422] = "post.xx4 = string";
	m_schema[423] = "post.xx40 = string";
	m_schema[424] = "post.xx41 = string";
	m_schema[425] = "post.xx42 = string";
	m_schema[426] = "post.xx43 = string";
	m_schema[427] = "post.xx44 = string";
	m_schema[428] = "post.xx45 = string";
	m_schema[429] = "post.xx46 = string";
	m_schema[430] = "post.xx47 = string";
	m_schema[431] = "post.xx48 = string";
	m_schema[432] = "post.xx49 = string";
	m_schema[433] = "post.xx5 = string";
	m_schema[434] = "post.xx50 = string";
	m_schema[435] = "post.xx51 = string";
	m_schema[436] = "post.xx52 = string";
	m_schema[437] = "post.xx53 = string";
	m_schema[438] = "post.xx54 = string";
	m_schema[439] = "post.xx55 = string";
	m_schema[440] = "post.xx56 = string";
	m_schema[441] = "post.xx57 = string";
	m_schema[442] = "post.xx58 = string";
	m_schema[443] = "post.xx59 = string";
	m_schema[444] = "post.xx6 = string";
	m_schema[445] = "post.xx60 = string";
	m_schema[446] = "post.xx61 = string";
	m_schema[447] = "post.xx62 = string";
	m_schema[448] = "post.xx63 = string";
	m_schema[449] = "post.xx64 = string";
	m_schema[450] = "post.xx65 = string";
	m_schema[451] = "post.xx66 = string";
	m_schema[452] = "post.xx67 = string";
	m_schema[453] = "post.xx68 = string";
	m_schema[454] = "post.xx69 = string";
	m_schema[455] = "post.xx7 = string";
	m_schema[456] = "post.xx70 = string";
	m_schema[457] = "post.xx71 = string";
	m_schema[458] = "post.xx72 = string";
	m_schema[459] = "post.xx73 = string";
	m_schema[460] = "post.xx74 = string";
	m_schema[461] = "post.xx75 = string";
	m_schema[462] = "post.xx76 = string";
	m_schema[463] = "post.xx77 = string";
	m_schema[464] = "post.xx78 = string";
	m_schema[465] = "post.xx79 = string";
	m_schema[466] = "post.xx8 = string";
	m_schema[467] = "post.xx80 = string";
	m_schema[468] = "post.xx81 = string";
	m_schema[469] = "post.xx82 = string";
	m_schema[470] = "post.xx83 = string";
	m_schema[471] = "post.xx84 = string";
	m_schema[472] = "post.xx85 = string";
	m_schema[473] = "post.xx86 = string";
	m_schema[474] = "post.xx87 = string";
	m_schema[475] = "post.xx88 = string";
	m_schema[476] = "post.xx89 = string";
	m_schema[477] = "post.xx9 = string";
	m_schema[478] = "post.xx90 = string";
	m_schema[479] = "post.xx91 = string";
	m_schema[480] = "post.xx92 = string";
	m_schema[481] = "post.xx93 = string";
	m_schema[482] = "post.xx94 = string";
	m_schema[483] = "post.xx95 = string";
	m_schema[484] = "post.xx96 = string";
	m_schema[485] = "post.xx97 = string";
	m_schema[486] = "post.xx98 = string";
	m_schema[487] = "post.xx99 = string";
	m_schema[488] = "project_name = string";
	m_schema[489] = "safe = scope";
	m_schema[490] = "safe.count = int";
	m_schema[491] = "safe.s1 = scope";
	m_schema[492] = "safe.s1.h = string";
	m_schema[493] = "safe.s1.type = string";
	m_schema[494] = "safe.s10 = scope";
	m_schema[495] = "safe.s10.h = string";
	m_schema[496] = "safe.s10.type = string";
	m_schema[497] = "safe.s11 = scope";
	m_schema[498] = "safe.s11.h = string";
	m_schema[499] = "safe.s11.type = string";
	m_schema[500] = "safe.s12 = scope";
	m_schema[501] = "safe.s12.h = string";
	m_schema[502] = "safe.s12.type = string";
	m_schema[503] = "safe.s13 = scope";
	m_schema[504] = "safe.s13.h = string";
	m_schema[505] = "safe.s13.type = string";
	m_schema[506] = "safe.s14 = scope";
	m_schema[507] = "safe.s14.h = string";
	m_schema[508] = "safe.s14.type = string";
	m_schema[509] = "safe.s15 = scope";
	m_schema[510] = "safe.s15.h = string";
	m_schema[511] = "safe.s15.type = string";
	m_schema[512] = "safe.s16 = scope";
	m_schema[513] = "safe.s16.h = string";
	m_schema[514] = "safe.s16.type = string";
	m_schema[515] = "safe.s17 = scope";
	m_schema[516] = "safe.s17.h = string";
	m_schema[517] = "safe.s17.type = string";
	m_schema[518] = "safe.s18 = scope";
	m_schema[519] = "safe.s18.h = string";
	m_schema[520] = "safe.s18.type = string";
	m_schema[521] = "safe.s19 = scope";
	m_schema[522] = "safe.s19.h = string";
	m_schema[523] = "safe.s19.type = string";
	m_schema[524] = "safe.s2 = scope";
	m_schema[525] = "safe.s2.h = string";
	m_schema[526] = "safe.s2.type = string";
	m_schema[527] = "safe.s20 = scope";
	m_schema[528] = "safe.s20.h = string";
	m_schema[529] = "safe.s20.type = string";
	m_schema[530] = "safe.s21 = scope";
	m_schema[531] = "safe.s21.h = string";
	m_schema[532] = "safe.s21.type = string";
	m_schema[533] = "safe.s22 = scope";
	m_schema[534] = "safe.s22.h = string";
	m_schema[535] = "safe.s22.type = string";
	m_schema[536] = "safe.s23 = scope";
	m_schema[537] = "safe.s23.h = string";
	m_schema[538] = "safe.s23.type = string";
	m_schema[539] = "safe.s24 = scope";
	m_schema[540] = "safe.s24.h = string";
	m_schema[541] = "safe.s24.type = string";
	m_schema[542] = "safe.s25 = scope";
	m_schema[543] = "safe.s25.h = string";
	m_schema[544] = "safe.s25.type = string";
	m_schema[545] = "safe.s26 = scope";
	m_schema[546] = "safe.s26.h = string";
	m_schema[547] = "safe.s26.type = string";
	m_schema[548] = "safe.s27 = scope";
	m_schema[549] = "safe.s27.h = string";
	m_schema[550] = "safe.s27.type = string";
	m_schema[551] = "safe.s28 = scope";
	m_schema[552] = "safe.s28.h = string";
	m_schema[553] = "safe.s28.type = string";
	m_schema[554] = "safe.s29 = scope";
	m_schema[555] = "safe.s29.h = string";
	m_schema[556] = "safe.s29.type = string";
	m_schema[557] = "safe.s3 = scope";
	m_schema[558] = "safe.s3.h = string";
	m_schema[559] = "safe.s3.type = string";
	m_schema[560] = "safe.s30 = scope";
	m_schema[561] = "safe.s30.h = string";
	m_schema[562] = "safe.s30.type = string";
	m_schema[563] = "safe.s31 = scope";
	m_schema[564] = "safe.s31.h = string";
	m_schema[565] = "safe.s31.type = string";
	m_schema[566] = "safe.s32 = scope";
	m_schema[567] = "safe.s32.h = string";
	m_schema[568] = "safe.s32.type = string";
	m_schema[569] = "safe.s33 = scope";
	m_schema[570] = "safe.s33.h = string";
	m_schema[571] = "safe.s33.type = string";
	m_schema[572] = "safe.s34 = scope";
	m_schema[573] = "safe.s34.h = string";
	m_schema[574] = "safe.s34.type = string";
	m_schema[575] = "safe.s35 = scope";
	m_schema[576] = "safe.s35.h = string";
	m_schema[577] = "safe.s35.type = string";
	m_schema[578] = "safe.s36 = scope";
	m_schema[579] = "safe.s36.h = string";
	m_schema[580] = "safe.s36.type = string";
	m_schema[581] = "safe.s37 = scope";
	m_schema[582] = "safe.s37.h = string";
	m_schema[583] = "safe.s37.type = string";
	m_schema[584] = "safe.s38 = scope";
	m_schema[585] = "safe.s38.h = string";
	m_schema[586] = "safe.s38.type = string";
	m_schema[587] = "safe.s4 = scope";
	m_schema[588] = "safe.s4.h = string";
	m_schema[589] = "safe.s4.type = string";
	m_schema[590] = "safe.s40 = scope";
	m_schema[591] = "safe.s40.h = string";
	m_schema[592] = "safe.s40.type = string";
	m_schema[593] = "safe.s41 = scope";
	m_schema[594] = "safe.s41.h = string";
	m_schema[595] = "safe.s41.type = string";
	m_schema[596] = "safe.s42 = scope";
	m_schema[597] = "safe.s42.h = string";
	m_schema[598] = "safe.s42.type = string";
	m_schema[599] = "safe.s43 = scope";
	m_schema[600] = "safe.s43.h = string";
	m_schema[601] = "safe.s43.type = string";
	m_schema[602] = "safe.s44 = scope";
	m_schema[603] = "safe.s44.h = string";
	m_schema[604] = "safe.s44.type = string";
	m_schema[605] = "safe.s45 = scope";
	m_schema[606] = "safe.s45.h = string";
	m_schema[607] = "safe.s45.type = string";
	m_schema[608] = "safe.s46 = scope";
	m_schema[609] = "safe.s46.h = string";
	m_schema[610] = "safe.s46.type = string";
	m_schema[611] = "safe.s47 = scope";
	m_schema[612] = "safe.s47.h = string";
	m_schema[613] = "safe.s47.type = string";
	m_schema[614] = "safe.s48 = scope";
	m_schema[615] = "safe.s48.h = string";
	m_schema[616] = "safe.s48.type = string";
	m_schema[617] = "safe.s49 = scope";
	m_schema[618] = "safe.s49.h = string";
	m_schema[619] = "safe.s49.type = string";
	m_schema[620] = "safe.s5 = scope";
	m_schema[621] = "safe.s5.h = string";
	m_schema[622] = "safe.s5.type = string";
	m_schema[623] = "safe.s50 = scope";
	m_schema[624] = "safe.s50.h = string";
	m_schema[625] = "safe.s50.type = string";
	m_schema[626] = "safe.s6 = scope";
	m_schema[627] = "safe.s6.h = string";
	m_schema[628] = "safe.s6.type = string";
	m_schema[629] = "safe.s7 = scope";
	m_schema[630] = "safe.s7.h = string";
	m_schema[631] = "safe.s7.type = string";
	m_schema[632] = "safe.s8 = scope";
	m_schema[633] = "safe.s8.h = string";
	m_schema[634] = "safe.s8.type = string";
	m_schema[635] = "safe.s9 = scope";
	m_schema[636] = "safe.s9.h = string";
	m_schema[637] = "safe.s9.type = string";
	m_schema[638] = "sampling_period = float";
	m_schema[639] = "save_controller = boolean";
	m_schema[640] = "save_transitions = boolean";
	m_schema[641] = "states = scope";
	m_schema[642] = "states.dim = int";
	m_schema[643] = "states.err = string";
	m_schema[644] = "states.eta = string";
	m_schema[645] = "states.lb = string";
	m_schema[646] = "states.ub = string";
	m_schema[647] = "synthesis_pack = string";
	m_schema[648] = "target = scope";
	m_schema[649] = "target.count = int";
	m_schema[650] = "target.t1 = scope";
	m_schema[651] = "target.t1.h = string";
	m_schema[652] = "target.t1.type = string";
	m_schema[653] = "target.t10 = scope";
	m_schema[654] = "target.t10.h = string";
	m_schema[655] = "target.t10.type = string";
	m_schema[656] = "target.t11 = scope";
	m_schema[657] = "target.t11.h = string";
	m_schema[658] = "target.t11.type = string";
	m_schema[659] = "target.t12 = scope";
	m_schema[660] = "target.t12.h = string";
	m_schema[661] = "target.t12.type = string";
	m_schema[662] = "target.t13 = scope";
	m_schema[663] = "target.t13.h = string";
	m_schema[664] = "target.t13.type = string";
	m_schema[665] = "target.t14 = scope";
	m_schema[666] = "target.t14.h = string";
	m_schema[667] = "target.t14.type = string";
	m_schema[668] = "target.t15 = scope";
	m_schema[669] = "target.t15.h = string";
	m_schema[670] = "target.t15.type = string";
	m_schema[671] = "target.t16 = scope";
	m_schema[672] = "target.t16.h = string";
	m_schema[673] = "target.t16.type = string";
	m_schema[674] = "target.t17 = scope";
	m_schema[675] = "target.t17.h = string";
	m_schema[676] = "target.t17.type = string";
	m_schema[677] = "target.t18 = scope";
	m_schema[678] = "target.t18.h = string";
	m_schema[679] = "target.t18.type = string";
	m_schema[680] = "target.t19 = scope";
	m_schema[681] = "target.t19.h = string";
	m_schema[682] = "target.t19.type = string";
	m_schema[683] = "target.t2 = scope";
	m_schema[684] = "target.t2.h = string";
	m_schema[685] = "target.t2.type = string";
	m_schema[686] = "target.t20 = scope";
	m_schema[687] = "target.t20.h = string";
	m_schema[688] = "target.t20.type = string";
	m_schema[689] = "target.t21 = scope";
	m_schema[690] = "target.t21.h = string";
	m_schema[691] = "target.t21.type = string";
	m_schema[692] = "target.t22 = scope";
	m_schema[693] = "target.t22.h = string";
	m_schema[694] = "target.t22.type = string";
	m_schema[695] = "target.t23 = scope";
	m_schema[696] = "target.t23.h = string";
	m_schema[697] = "target.t23.type = string";
	m_schema[698] = "target.t24 = scope";
	m_schema[699] = "target.t24.h = string";
	m_schema[700] = "target.t24.type = string";
	m_schema[701] = "target.t25 = scope";
	m_schema[702] = "target.t25.h = string";
	m_schema[703] = "target.t25.type = string";
	m_schema[704] = "target.t26 = scope";
	m_schema[705] = "target.t26.h = string";
	m_schema[706] = "target.t26.type = string";
	m_schema[707] = "target.t27 = scope";
	m_schema[708] = "target.t27.h = string";
	m_schema[709] = "target.t27.type = string";
	m_schema[710] = "target.t28 = scope";
	m_schema[711] = "target.t28.h = string";
	m_schema[712] = "target.t28.type = string";
	m_schema[713] = "target.t29 = scope";
	m_schema[714] = "target.t29.h = string";
	m_schema[715] = "target.t29.type = string";
	m_schema[716] = "target.t3 = scope";
	m_schema[717] = "target.t3.h = string";
	m_schema[718] = "target.t3.type = string";
	m_schema[719] = "target.t30 = scope";
	m_schema[720] = "target.t30.h = string";
	m_schema[721] = "target.t30.type = string";
	m_schema[722] = "target.t31 = scope";
	m_schema[723] = "target.t31.h = string";
	m_schema[724] = "target.t31.type = string";
	m_schema[725] = "target.t32 = scope";
	m_schema[726] = "target.t32.h = string";
	m_schema[727] = "target.t32.type = string";
	m_schema[728] = "target.t33 = scope";
	m_schema[729] = "target.t33.h = string";
	m_schema[730] = "target.t33.type = string";
	m_schema[731] = "target.t34 = scope";
	m_schema[732] = "target.t34.h = string";
	m_schema[733] = "target.t34.type = string";
	m_schema[734] = "target.t35 = scope";
	m_schema[735] = "target.t35.h = string";
	m_schema[736] = "target.t35.type = string";
	m_schema[737] = "target.t36 = scope";
	m_schema[738] = "target.t36.h = string";
	m_schema[739] = "target.t36.type = string";
	m_schema[740] = "target.t37 = scope";
	m_schema[741] = "target.t37.h = string";
	m_schema[742] = "target.t37.type = string";
	m_schema[743] = "target.t38 = scope";
	m_schema[744] = "target.t38.h = string";
	m_schema[745] = "target.t38.type = string";
	m_schema[746] = "target.t39 = scope";
	m_schema[747] = "target.t39.h = string";
	m_schema[748] = "target.t39.type = string";
	m_schema[749] = "target.t4 = scope";
	m_schema[750] = "target.t4.h = string";
	m_schema[751] = "target.t4.type = string";
	m_schema[752] = "target.t40 = scope";
	m_schema[753] = "target.t40.h = string";
	m_schema[754] = "target.t40.type = string";
	m_schema[755] = "target.t41 = scope";
	m_schema[756] = "target.t41.h = string";
	m_schema[757] = "target.t41.type = string";
	m_schema[758] = "target.t42 = scope";
	m_schema[759] = "target.t42.h = string";
	m_schema[760] = "target.t42.type = string";
	m_schema[761] = "target.t43 = scope";
	m_schema[762] = "target.t43.h = string";
	m_schema[763] = "target.t43.type = string";
	m_schema[764] = "target.t44 = scope";
	m_schema[765] = "target.t44.h = string";
	m_schema[766] = "target.t44.type = string";
	m_schema[767] = "target.t45 = scope";
	m_schema[768] = "target.t45.h = string";
	m_schema[769] = "target.t45.type = string";
	m_schema[770] = "target.t46 = scope";
	m_schema[771] = "target.t46.h = string";
	m_schema[772] = "target.t46.type = string";
	m_schema[773] = "target.t47 = scope";
	m_schema[774] = "target.t47.h = string";
	m_schema[775] = "target.t47.type = string";
	m_schema[776] = "target.t48 = scope";
	m_schema[777] = "target.t48.h = string";
	m_schema[778] = "target.t48.type = string";
	m_schema[779] = "target.t49 = scope";
	m_schema[780] = "target.t49.h = string";
	m_schema[781] = "target.t49.type = string";
	m_schema[782] = "target.t5 = scope";
	m_schema[783] = "target.t5.h = string";
	m_schema[784] = "target.t5.type = string";
	m_schema[785] = "target.t50 = scope";
	m_schema[786] = "target.t50.h = string";
	m_schema[787] = "target.t50.type = string";
	m_schema[788] = "target.t6 = scope";
	m_schema[789] = "target.t6.h = string";
	m_schema[790] = "target.t6.type = string";
	m_schema[791] = "target.t7 = scope";
	m_schema[792] = "target.t7.h = string";
	m_schema[793] = "target.t7.type = string";
	m_schema[794] = "target.t8 = scope";
	m_schema[795] = "target.t8.h = string";
	m_schema[796] = "target.t8.type = string";
	m_schema[797] = "target.t9 = scope";
	m_schema[798] = "target.t9.h = string";
	m_schema[799] = "target.t9.type = string";
	m_schema[800] = "extra_include_file = string";
	m_schema[801] = "disturbances = scope";
	m_schema[802] = "disturbances.dim = int";
	m_schema[803] = "user_dynamics_file = string";
	m_schema[804] = "ode_steps = int";
	m_schema[805] = "max_basis_elements = int";
	m_schema[806] = "record_basis_evolution = boolean";
	m_schema[807] = "states.priorities = string";
	m_schema[808] = "benchmark_count = int";
	m_schema[809] = "synthesis_method = string";
	m_schema[810] = "transition_semantics = string";
	m_schema[811] = "transition_backend = string";
	m_schema[812] = "threshold_d_star = int";
	m_schema[813] = "extract_basis = boolean";
	// Accepted only so validate_values can issue a precise migration error.
	m_schema[814] = "use_threshold_table = string";
	m_schema[815] = "use_tt_only = string";
	m_schema[816] = "use_tt_only_gpu = string";
	m_schema[817] = "use_bitmap_gfp = string";
	m_schema[818] = "use_inline_dynamics = string";
	m_schema[819] = "use_prefix_sweep = string";
	m_schema[820] = "boundary_seeding = string";
	m_schema[821] = "boundary_semantics = string";
	m_schema[822] = 0;


	std::stringstream m_str;
	m_str << "#-------------------------------------------------";
	m_str << "----------------------\n";
	m_str << "# File:        pfacesDefaultConfiguration.cfg\n";
	m_str << "#-------------------------------------------------";
	m_str << "----------------------\n";
	m_str << "# Author: \n";
	m_str << "# Date created: \n";
	m_str << "# Description: A configuration file for pFaces to ";
	m_str << "do ...\n";
	m_str << "#-------------------------------------------------";
	m_str << "----------------------\n";
	m_str << "\n";
	m_str << "# Project data\n";
	m_str << "# -------------------------\n";
	m_str << "# (Project name) is essential. It will be used for";
	m_str << " names of output files.\n";
	m_str << "# (synthesis_pack) is optional and defines how man";
	m_str << "y synthesis tasks are paked together\n";
	m_str << "#  and its values can be :\n";
	m_str << "#\t1- off: meaning one synthesis task at a time\n";
	m_str << "#\t2- x%%: where x is positive value between 0 and ";
	m_str << "99 as percentage of the domain diameter [0% = size";
	m_str << " 1].\n";
	m_str << "#\t3- x  : where x is non zero positive value repre";
	m_str << "senting exact number of packed synthesis tasks.\n";
	m_str << "project_name = \"xxxx\";\n";
	m_str << "extra_include_file = \"xxxx\";\n";

	m_str << "synthesis_pack = \"off\";\n";
	m_str << "online_mode = \"false\";\n";
	m_str << "\n";
	m_str << "# Output Memory Model\n";
	m_str << "# -------------------------\n";
	m_str << "# The (data) tells the tool which memory model to ";
	m_str << "use when saving results\n";
	m_str << "# use the values : raw | bits | bdd\n";
	m_str << "data = \"raw\";\n";
	m_str << "save_transitions = \"false\";\n";
	m_str << "save_controller = \"true\";\n";
	m_str << "\n";
	m_str << "# Code Generation\n";
	m_str << "# -------------------------\n";
	m_str << "# The (implement)  tells the tool which implememta";
	m_str << "tion is used for code generation\n";
	m_str << "# use the values : no | cpp | c | vhdl | verilog\n";
	m_str << "implement = \"cpp\";\n";
	m_str << "\n";
	m_str << "\n";
	m_str << "# Sampling period\n";
	m_str << "# -------------------------\n";
	m_str << "sampling_period = \"0.0\";\n";
	m_str << "user_dynamics_file = \"xxxx\";\n";
	m_str << "ode_steps = \"100\";\n";
	m_str << "max_basis_elements = \"10000\";\n";
	m_str << "record_basis_evolution = \"false\";\n";
	m_str << "benchmark_count = \"10\";\n";
	m_str << "synthesis_method = \"threshold\";\n";
	m_str << "transition_semantics = \"extremal_single_successor\";\n";
	m_str << "transition_backend = \"precomputed\";\n";
	m_str << "boundary_semantics = \"strict_unsafe\";\n";
	m_str << "threshold_d_star = \"-1\";\n";
	m_str << "extract_basis = \"true\";\n";
	// Sentinels let the driver distinguish an absent legacy key from any
	// explicitly supplied legacy value while satisfying Config4Cpp's schema.
	m_str << "use_threshold_table = \"__mono_synth_legacy_unset__\";\n";
	m_str << "use_tt_only = \"__mono_synth_legacy_unset__\";\n";
	m_str << "use_tt_only_gpu = \"__mono_synth_legacy_unset__\";\n";
	m_str << "use_bitmap_gfp = \"__mono_synth_legacy_unset__\";\n";
	m_str << "use_inline_dynamics = \"__mono_synth_legacy_unset__\";\n";
	m_str << "use_prefix_sweep = \"__mono_synth_legacy_unset__\";\n";
	m_str << "boundary_seeding = \"__mono_synth_legacy_unset__\";\n";
	m_str << "\n";
	m_str << "\n";
	m_str << "# State/Input sets\n";
	m_str << "# -------------------------\n";
	m_str << "states{\n";
	m_str << "\tdim = \"0\";\n";
	m_str << "\teta = \"xxxx\";\n";
	m_str << "\tlb  = \"xxxx\";\n";
	m_str << "\tub  = \"xxxx\";\n";
	m_str << "\terr = \"xxxx\";\n";
	m_str << "\tpriorities = \"xxxx\";\n";
	m_str << "}\n";
	m_str << "inputs{\n";
	m_str << "\tdim = \"0\";\n";
	m_str << "\teta = \"xxxx\";\n";
	m_str << "\tlb  = \"xxxx\";\n";
	m_str << "\tub  = \"xxxx\";\n";
	m_str << "\terr = \"xxxx\";\n";
	m_str << "}\n";
	m_str << "disturbances{\n";
	m_str << "\tdim = \"0\";\n";
	m_str << "}\n";
	m_str << "\n";
	m_str << "# System-post dynamics\n";
	m_str << "# -------------------------\n";
	m_str << "# Your post variables should start with (xx) follo";
	m_str << "wed by the index of the dimension (starting from 1";
	m_str << " not 0).\n";
	m_str << "# Maximum number of dimensions is (states.dim)\n";
	m_str << "# If useode is specified, the ODE solver will be u";
	m_str << "sed to solve the dynamics.\n";
	m_str << "# Using initcode_XX (with XX from 1 to 30), you ca";
	m_str << "n define some initialization code or variables \n";
	m_str << "# to be used inside the dynamics equations.\n";
	m_str << "# In dynamics, you are allowed to use:\n";
	m_str << "#  - array indexing to access states in x (e.g., x";
	m_str << "1 as the first state)\n";
	m_str << "#  - array indexing to access inputs in u (e.g., u";
	m_str << "1 as the first input)\n";
	m_str << "#  - any math function from: https://www.khronos.o";
	m_str << "rg/registry/OpenCL/sdk/1.0/docs/man/xhtml/mathFunc";
	m_str << "tions.html\n";
	m_str << "post{\n";
	m_str << "\tuseode = \"false\";\n";
	m_str << "\tcodeonly = \"false\";\n";
	m_str << "\t\n";
	m_str << "    initcode_1  = \"opencl-code\";\n";
	m_str << "    initcode_2  = \"opencl-code\";\n";
	m_str << "\tinitcode_3  = \"opencl-code\";\n";
	m_str << "\tinitcode_4  = \"opencl-code\";\n";
	m_str << "\tinitcode_5  = \"opencl-code\";\n";
	m_str << "\tinitcode_6  = \"opencl-code\";\n";
	m_str << "\tinitcode_7  = \"opencl-code\";\n";
	m_str << "\tinitcode_8  = \"opencl-code\";\n";
	m_str << "\tinitcode_9  = \"opencl-code\";\n";
	m_str << "\tinitcode_10 = \"opencl-code\";\n";
	m_str << "\tinitcode_11 = \"opencl-code\";\n";
	m_str << "\tinitcode_12 = \"opencl-code\";\n";
	m_str << "\tinitcode_13 = \"opencl-code\";\n";
	m_str << "\tinitcode_14 = \"opencl-code\";\n";
	m_str << "\tinitcode_15 = \"opencl-code\";\n";
	m_str << "\tinitcode_16 = \"opencl-code\";\n";
	m_str << "\tinitcode_17 = \"opencl-code\";\n";
	m_str << "\tinitcode_18 = \"opencl-code\";\n";
	m_str << "\tinitcode_19 = \"opencl-code\";\n";
	m_str << "\tinitcode_20 = \"opencl-code\";\n";
	m_str << "\tinitcode_21 = \"opencl-code\";\n";
	m_str << "\tinitcode_22 = \"opencl-code\";\n";
	m_str << "\tinitcode_23 = \"opencl-code\";\n";
	m_str << "\tinitcode_24 = \"opencl-code\";\n";
	m_str << "\tinitcode_25 = \"opencl-code\";\n";
	m_str << "\tinitcode_26 = \"opencl-code\";\n";
	m_str << "\tinitcode_27 = \"opencl-code\";\n";
	m_str << "\tinitcode_28 = \"opencl-code\";\n";
	m_str << "\tinitcode_29 = \"opencl-code\";\n";
	m_str << "\tinitcode_30 = \"opencl-code\";\n";
	m_str << "\t\n";
	m_str << "\txx1 = \"EMPTY\";\n";
	m_str << "\txx2 = \"EMPTY\";\n";
	m_str << "\txx3 = \"EMPTY\";\n";
	m_str << "\txx4 = \"EMPTY\";\n";
	m_str << "\txx5 = \"EMPTY\";\n";
	m_str << "\txx6 = \"EMPTY\";\n";
	m_str << "\txx7 = \"EMPTY\";\n";
	m_str << "\txx8 = \"EMPTY\";\n";
	m_str << "\txx9 = \"EMPTY\";\n";
	m_str << "\txx10 = \"EMPTY\";\n";
	m_str << "\txx11 = \"EMPTY\";\n";
	m_str << "\txx12 = \"EMPTY\";\n";
	m_str << "\txx13 = \"EMPTY\";\n";
	m_str << "\txx14 = \"EMPTY\";\n";
	m_str << "\txx15 = \"EMPTY\";\n";
	m_str << "\txx16 = \"EMPTY\";\n";
	m_str << "\txx17 = \"EMPTY\";\n";
	m_str << "\txx18 = \"EMPTY\";\n";
	m_str << "\txx19 = \"EMPTY\";\n";
	m_str << "\txx20 = \"EMPTY\";\n";
	m_str << "\txx21 = \"EMPTY\";\n";
	m_str << "\txx22 = \"EMPTY\";\n";
	m_str << "\txx23 = \"EMPTY\";\n";
	m_str << "\txx24 = \"EMPTY\";\n";
	m_str << "\txx25 = \"EMPTY\";\n";
	m_str << "\txx26 = \"EMPTY\";\n";
	m_str << "\txx27 = \"EMPTY\";\n";
	m_str << "\txx28 = \"EMPTY\";\n";
	m_str << "\txx29 = \"EMPTY\";\n";
	m_str << "\txx30 = \"EMPTY\";\t\n";
	m_str << "\txx31 = \"EMPTY\";\n";
	m_str << "\txx32 = \"EMPTY\";\n";
	m_str << "\txx33 = \"EMPTY\";\n";
	m_str << "\txx34 = \"EMPTY\";\n";
	m_str << "\txx35 = \"EMPTY\";\n";
	m_str << "\txx36 = \"EMPTY\";\n";
	m_str << "\txx37 = \"EMPTY\";\n";
	m_str << "\txx38 = \"EMPTY\";\n";
	m_str << "\txx39 = \"EMPTY\";\n";
	m_str << "\txx40 = \"EMPTY\";\t\n";
	m_str << "\txx41 = \"EMPTY\";\n";
	m_str << "\txx42 = \"EMPTY\";\n";
	m_str << "\txx43 = \"EMPTY\";\n";
	m_str << "\txx44 = \"EMPTY\";\n";
	m_str << "\txx45 = \"EMPTY\";\n";
	m_str << "\txx46 = \"EMPTY\";\n";
	m_str << "\txx47 = \"EMPTY\";\n";
	m_str << "\txx48 = \"EMPTY\";\n";
	m_str << "\txx49 = \"EMPTY\";\n";
	m_str << "\txx50 = \"EMPTY\";\t\n";
	m_str << "\txx51 = \"EMPTY\";\n";
	m_str << "\txx52 = \"EMPTY\";\n";
	m_str << "\txx53 = \"EMPTY\";\n";
	m_str << "\txx54 = \"EMPTY\";\n";
	m_str << "\txx55 = \"EMPTY\";\n";
	m_str << "\txx56 = \"EMPTY\";\n";
	m_str << "\txx57 = \"EMPTY\";\n";
	m_str << "\txx58 = \"EMPTY\";\n";
	m_str << "\txx59 = \"EMPTY\";\n";
	m_str << "\txx60 = \"EMPTY\";\t\n";
	m_str << "\txx61 = \"EMPTY\";\n";
	m_str << "\txx62 = \"EMPTY\";\n";
	m_str << "\txx63 = \"EMPTY\";\n";
	m_str << "\txx64 = \"EMPTY\";\n";
	m_str << "\txx65 = \"EMPTY\";\n";
	m_str << "\txx66 = \"EMPTY\";\n";
	m_str << "\txx67 = \"EMPTY\";\n";
	m_str << "\txx68 = \"EMPTY\";\n";
	m_str << "\txx69 = \"EMPTY\";\n";
	m_str << "\txx70 = \"EMPTY\";\t\n";
	m_str << "\txx71 = \"EMPTY\";\n";
	m_str << "\txx72 = \"EMPTY\";\n";
	m_str << "\txx73 = \"EMPTY\";\n";
	m_str << "\txx74 = \"EMPTY\";\n";
	m_str << "\txx75 = \"EMPTY\";\n";
	m_str << "\txx76 = \"EMPTY\";\n";
	m_str << "\txx77 = \"EMPTY\";\n";
	m_str << "\txx78 = \"EMPTY\";\n";
	m_str << "\txx79 = \"EMPTY\";\n";
	m_str << "\txx80 = \"EMPTY\";\t\n";
	m_str << "\txx81 = \"EMPTY\";\n";
	m_str << "\txx82 = \"EMPTY\";\n";
	m_str << "\txx83 = \"EMPTY\";\n";
	m_str << "\txx84 = \"EMPTY\";\n";
	m_str << "\txx85 = \"EMPTY\";\n";
	m_str << "\txx86 = \"EMPTY\";\n";
	m_str << "\txx87 = \"EMPTY\";\n";
	m_str << "\txx88 = \"EMPTY\";\n";
	m_str << "\txx89 = \"EMPTY\";\n";
	m_str << "\txx90 = \"EMPTY\";\t\n";
	m_str << "\txx91 = \"EMPTY\";\n";
	m_str << "\txx92 = \"EMPTY\";\n";
	m_str << "\txx93 = \"EMPTY\";\n";
	m_str << "\txx94 = \"EMPTY\";\n";
	m_str << "\txx95 = \"EMPTY\";\n";
	m_str << "\txx96 = \"EMPTY\";\n";
	m_str << "\txx97 = \"EMPTY\";\n";
	m_str << "\txx98 = \"EMPTY\";\n";
	m_str << "\txx99 = \"EMPTY\";\n";
	m_str << "\txx100 = \"EMPTY\";\t\n";
	m_str << "\n";
	m_str << "    finishcode_1  = \"opencl-code\";\n";
	m_str << "\tfinishcode_2  = \"opencl-code\";\n";
	m_str << "\tfinishcode_3  = \"opencl-code\";\n";
	m_str << "\tfinishcode_4  = \"opencl-code\";\n";
	m_str << "\tfinishcode_5  = \"opencl-code\";\n";
	m_str << "\tfinishcode_6  = \"opencl-code\";\n";
	m_str << "\tfinishcode_7  = \"opencl-code\";\n";
	m_str << "\tfinishcode_8  = \"opencl-code\";\n";
	m_str << "\tfinishcode_9  = \"opencl-code\";\n";
	m_str << "\tfinishcode_10  = \"opencl-code\";\n";
	m_str << "\tfinishcode_11  = \"opencl-code\";\n";
	m_str << "\tfinishcode_12  = \"opencl-code\";\n";
	m_str << "\tfinishcode_13  = \"opencl-code\";\n";
	m_str << "\tfinishcode_14  = \"opencl-code\";\n";
	m_str << "\tfinishcode_15  = \"opencl-code\";\n";
	m_str << "\tfinishcode_16  = \"opencl-code\";\n";
	m_str << "\tfinishcode_17  = \"opencl-code\";\n";
	m_str << "\tfinishcode_18  = \"opencl-code\";\n";
	m_str << "\tfinishcode_19  = \"opencl-code\";\n";
	m_str << "\tfinishcode_20  = \"opencl-code\";\n";
	m_str << "\tfinishcode_21  = \"opencl-code\";\n";
	m_str << "\tfinishcode_22  = \"opencl-code\";\n";
	m_str << "\tfinishcode_23  = \"opencl-code\";\n";
	m_str << "\tfinishcode_24  = \"opencl-code\";\n";
	m_str << "\tfinishcode_25  = \"opencl-code\";\n";
	m_str << "\tfinishcode_26  = \"opencl-code\";\n";
	m_str << "\tfinishcode_27  = \"opencl-code\";\n";
	m_str << "\tfinishcode_28  = \"opencl-code\";\n";
	m_str << "\tfinishcode_29  = \"opencl-code\";\n";
	m_str << "\tfinishcode_30  = \"opencl-code\";\n";
	m_str << "}\n";
	m_str << "\n";
	m_str << "# System-growth dynamics\n";
	m_str << "# -------------------------\n";
	m_str << "# Your growth variables should start with (rr) fol";
	m_str << "lowed by the index of the dimension (starting from";
	m_str << " 1 not 0).\n";
	m_str << "# Maximum number of dimensions is (states.dim)\n";
	m_str << "# If 'useode' is specified, the ODE solver will be";
	m_str << " used to solve the dynamics.\n";
	m_str << "# The 'maxposts' limits the iteration over post st";
	m_str << "ates in OARS and its default value is 64.\n";
	m_str << "# Using initcode_XX (with XX from 1 to 30), you ca";
	m_str << "n define some initialization code or variables \n";
	m_str << "# to be used inside the dynamics equations.\n";
	m_str << "# In dynamics, you are allowed to use:\n";
	m_str << "#  - array indexing to access radius in r (e.g., r";
	m_str << "1 as the first radius)\n";
	m_str << "#  - array indexing to access inputs in u (e.g., u";
	m_str << "1 as the first input)\n";
	m_str << "#  - any math function from: https://www.khronos.o";
	m_str << "rg/registry/OpenCL/sdk/1.0/docs/man/xhtml/mathFunc";
	m_str << "tions.html\n";
	m_str << "growth{\n";
	m_str << "\tuseode = \"false\";\n";
	m_str << "\tcodeonly = \"false\";\n";
	m_str << "\tmaxposts = \"64\";\n";
	m_str << "\t\n";
	m_str << "    initcode_1  = \"opencl-code\";\n";
	m_str << "    initcode_2  = \"opencl-code\";\n";
	m_str << "\tinitcode_3  = \"opencl-code\";\n";
	m_str << "\tinitcode_4  = \"opencl-code\";\n";
	m_str << "\tinitcode_5  = \"opencl-code\";\n";
	m_str << "\tinitcode_6  = \"opencl-code\";\n";
	m_str << "\tinitcode_7  = \"opencl-code\";\n";
	m_str << "\tinitcode_8  = \"opencl-code\";\n";
	m_str << "\tinitcode_9  = \"opencl-code\";\n";
	m_str << "\tinitcode_10 = \"opencl-code\";\n";
	m_str << "\tinitcode_11 = \"opencl-code\";\n";
	m_str << "\tinitcode_12 = \"opencl-code\";\n";
	m_str << "\tinitcode_13 = \"opencl-code\";\n";
	m_str << "\tinitcode_14 = \"opencl-code\";\n";
	m_str << "\tinitcode_15 = \"opencl-code\";\n";
	m_str << "\tinitcode_16 = \"opencl-code\";\n";
	m_str << "\tinitcode_17 = \"opencl-code\";\n";
	m_str << "\tinitcode_18 = \"opencl-code\";\n";
	m_str << "\tinitcode_19 = \"opencl-code\";\n";
	m_str << "\tinitcode_20 = \"opencl-code\";\n";
	m_str << "\tinitcode_21 = \"opencl-code\";\n";
	m_str << "\tinitcode_22 = \"opencl-code\";\n";
	m_str << "\tinitcode_23 = \"opencl-code\";\n";
	m_str << "\tinitcode_24 = \"opencl-code\";\n";
	m_str << "\tinitcode_25 = \"opencl-code\";\n";
	m_str << "\tinitcode_26 = \"opencl-code\";\n";
	m_str << "\tinitcode_27 = \"opencl-code\";\n";
	m_str << "\tinitcode_28 = \"opencl-code\";\n";
	m_str << "\tinitcode_29 = \"opencl-code\";\n";
	m_str << "\tinitcode_30 = \"opencl-code\";\t\n";
	m_str << "\t\n";
	m_str << "    rr1 = \"EMPTY\";\n";
	m_str << "    rr2 = \"EMPTY\";\t\n";
	m_str << "\trr3 = \"EMPTY\";\n";
	m_str << "\trr4 = \"EMPTY\";\n";
	m_str << "\trr5 = \"EMPTY\";\n";
	m_str << "\trr6 = \"EMPTY\";\n";
	m_str << "\trr7 = \"EMPTY\";\n";
	m_str << "\trr8 = \"EMPTY\";\n";
	m_str << "\trr9 = \"EMPTY\";\n";
	m_str << "\trr10 = \"EMPTY\";\n";
	m_str << "\trr11 = \"EMPTY\";\n";
	m_str << "\trr12 = \"EMPTY\";\n";
	m_str << "\trr13 = \"EMPTY\";\n";
	m_str << "\trr14 = \"EMPTY\";\n";
	m_str << "\trr15 = \"EMPTY\";\n";
	m_str << "\trr16 = \"EMPTY\";\n";
	m_str << "\trr17 = \"EMPTY\";\n";
	m_str << "\trr18 = \"EMPTY\";\n";
	m_str << "\trr19 = \"EMPTY\";\n";
	m_str << "\trr20 = \"EMPTY\";\n";
	m_str << "\trr21 = \"EMPTY\";\n";
	m_str << "\trr22 = \"EMPTY\";\n";
	m_str << "\trr23 = \"EMPTY\";\n";
	m_str << "\trr24 = \"EMPTY\";\n";
	m_str << "\trr25 = \"EMPTY\";\n";
	m_str << "\trr26 = \"EMPTY\";\n";
	m_str << "\trr27 = \"EMPTY\";\n";
	m_str << "\trr28 = \"EMPTY\";\n";
	m_str << "\trr29 = \"EMPTY\";\n";
	m_str << "\trr30 = \"EMPTY\";\n";
	m_str << "\trr31 = \"EMPTY\";\n";
	m_str << "\trr32 = \"EMPTY\";\n";
	m_str << "\trr33 = \"EMPTY\";\n";
	m_str << "\trr34 = \"EMPTY\";\n";
	m_str << "\trr35 = \"EMPTY\";\n";
	m_str << "\trr36 = \"EMPTY\";\n";
	m_str << "\trr37 = \"EMPTY\";\n";
	m_str << "\trr38 = \"EMPTY\";\n";
	m_str << "\trr39 = \"EMPTY\";\n";
	m_str << "\trr40 = \"EMPTY\";\n";
	m_str << "\trr41 = \"EMPTY\";\n";
	m_str << "\trr42 = \"EMPTY\";\n";
	m_str << "\trr43 = \"EMPTY\";\n";
	m_str << "\trr44 = \"EMPTY\";\n";
	m_str << "\trr45 = \"EMPTY\";\n";
	m_str << "\trr46 = \"EMPTY\";\n";
	m_str << "\trr47 = \"EMPTY\";\n";
	m_str << "\trr48 = \"EMPTY\";\n";
	m_str << "\trr49 = \"EMPTY\";\n";
	m_str << "\trr50 = \"EMPTY\";\n";
	m_str << "\trr51 = \"EMPTY\";\n";
	m_str << "\trr52 = \"EMPTY\";\n";
	m_str << "\trr53 = \"EMPTY\";\n";
	m_str << "\trr54 = \"EMPTY\";\n";
	m_str << "\trr55 = \"EMPTY\";\n";
	m_str << "\trr56 = \"EMPTY\";\n";
	m_str << "\trr57 = \"EMPTY\";\n";
	m_str << "\trr58 = \"EMPTY\";\n";
	m_str << "\trr59 = \"EMPTY\";\n";
	m_str << "\trr60 = \"EMPTY\";\n";
	m_str << "\trr61 = \"EMPTY\";\n";
	m_str << "\trr62 = \"EMPTY\";\n";
	m_str << "\trr63 = \"EMPTY\";\n";
	m_str << "\trr64 = \"EMPTY\";\n";
	m_str << "\trr65 = \"EMPTY\";\n";
	m_str << "\trr66 = \"EMPTY\";\n";
	m_str << "\trr67 = \"EMPTY\";\n";
	m_str << "\trr68 = \"EMPTY\";\n";
	m_str << "\trr69 = \"EMPTY\";\n";
	m_str << "\trr70 = \"EMPTY\";\n";
	m_str << "\trr71 = \"EMPTY\";\n";
	m_str << "\trr72 = \"EMPTY\";\n";
	m_str << "\trr73 = \"EMPTY\";\n";
	m_str << "\trr74 = \"EMPTY\";\n";
	m_str << "\trr75 = \"EMPTY\";\n";
	m_str << "\trr76 = \"EMPTY\";\n";
	m_str << "\trr77 = \"EMPTY\";\n";
	m_str << "\trr78 = \"EMPTY\";\n";
	m_str << "\trr79 = \"EMPTY\";\n";
	m_str << "\trr80 = \"EMPTY\";\n";
	m_str << "\trr81 = \"EMPTY\";\n";
	m_str << "\trr82 = \"EMPTY\";\n";
	m_str << "\trr83 = \"EMPTY\";\n";
	m_str << "\trr84 = \"EMPTY\";\n";
	m_str << "\trr85 = \"EMPTY\";\n";
	m_str << "\trr86 = \"EMPTY\";\n";
	m_str << "\trr87 = \"EMPTY\";\n";
	m_str << "\trr88 = \"EMPTY\";\n";
	m_str << "\trr89 = \"EMPTY\";\n";
	m_str << "\trr90 = \"EMPTY\";\n";
	m_str << "\trr91 = \"EMPTY\";\n";
	m_str << "\trr92 = \"EMPTY\";\n";
	m_str << "\trr93 = \"EMPTY\";\n";
	m_str << "\trr94 = \"EMPTY\";\n";
	m_str << "\trr95 = \"EMPTY\";\n";
	m_str << "\trr96 = \"EMPTY\";\n";
	m_str << "\trr97 = \"EMPTY\";\n";
	m_str << "\trr98 = \"EMPTY\";\n";
	m_str << "\trr99 = \"EMPTY\";\n";
	m_str << "\trr100 = \"EMPTY\";\n";
	m_str << "\n";
	m_str << "    finishcode_1  = \"opencl-code\";\n";
	m_str << "\tfinishcode_2  = \"opencl-code\";\n";
	m_str << "\tfinishcode_3  = \"opencl-code\";\n";
	m_str << "\tfinishcode_4  = \"opencl-code\";\n";
	m_str << "\tfinishcode_5  = \"opencl-code\";\n";
	m_str << "\tfinishcode_6  = \"opencl-code\";\n";
	m_str << "\tfinishcode_7  = \"opencl-code\";\n";
	m_str << "\tfinishcode_8  = \"opencl-code\";\n";
	m_str << "\tfinishcode_9  = \"opencl-code\";\n";
	m_str << "\tfinishcode_10  = \"opencl-code\";\n";
	m_str << "\tfinishcode_11  = \"opencl-code\";\n";
	m_str << "\tfinishcode_12  = \"opencl-code\";\n";
	m_str << "\tfinishcode_13  = \"opencl-code\";\n";
	m_str << "\tfinishcode_14  = \"opencl-code\";\n";
	m_str << "\tfinishcode_15  = \"opencl-code\";\n";
	m_str << "\tfinishcode_16  = \"opencl-code\";\n";
	m_str << "\tfinishcode_17  = \"opencl-code\";\n";
	m_str << "\tfinishcode_18  = \"opencl-code\";\n";
	m_str << "\tfinishcode_19  = \"opencl-code\";\n";
	m_str << "\tfinishcode_20  = \"opencl-code\";\n";
	m_str << "\tfinishcode_21  = \"opencl-code\";\n";
	m_str << "\tfinishcode_22  = \"opencl-code\";\n";
	m_str << "\tfinishcode_23  = \"opencl-code\";\n";
	m_str << "\tfinishcode_24  = \"opencl-code\";\n";
	m_str << "\tfinishcode_25  = \"opencl-code\";\n";
	m_str << "\tfinishcode_26  = \"opencl-code\";\n";
	m_str << "\tfinishcode_27  = \"opencl-code\";\n";
	m_str << "\tfinishcode_28  = \"opencl-code\";\n";
	m_str << "\tfinishcode_29  = \"opencl-code\";\n";
	m_str << "\tfinishcode_30  = \"opencl-code\";\n";
	m_str << "}\n";
	m_str << "\n";
	m_str << "# Obstacles for specifications\n";
	m_str << "# -----------------------------------------\n";
	m_str << "# Number of obstacles is specified in (count).\n";
	m_str << "# Your obstacles should start with (ob) followed b";
	m_str << "y an index (starting from 1 not 0).\n";
	m_str << "# Each obstacle is a scope and should contain its ";
	m_str << "type and required variables.\n";
	m_str << "# For ob#.type = \"rectangle\", specify the per-dime";
	m_str << "nsion bounds in (h) as a comma-sepaarated list.\n";
	m_str << "obstacles{\n";
	m_str << "\tcount=\"0\";\n";
	m_str << "\tob1{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob2{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob3{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob4{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob5{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob6{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob7{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob8{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob9{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob10{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob11{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob12{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob13{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob14{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob15{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob16{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob17{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob18{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob19{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob20{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob21{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob22{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob23{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob24{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob25{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob26{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob27{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob28{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob29{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob30{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob31{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob32{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob33{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob34{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob35{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob36{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob37{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob38{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob39{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob40{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob41{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob42{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob43{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob44{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob45{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob46{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob47{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob48{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob49{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tob50{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "}\n";
	m_str << "\n";
	m_str << "# A target for reach specifications\n";
	m_str << "# -----------------------------------------\n";
	m_str << "# For type = \"rectangle\", specify the per-dimensio";
	m_str << "n bounds in (h) as a comma-sepaarated list.\n";
	m_str << "target{ \n";
	m_str << "\tcount=\"0\"; \n";
	m_str << "\tt1{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt2{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt3{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt4{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt5{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt6{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt7{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt8{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt9{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt10{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt11{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt12{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt13{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt14{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt15{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt16{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt17{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt18{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt19{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt20{ type=\"xxxx\"; h=\"xxxx\";}\t\n";
	m_str << "\tt21{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt22{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt23{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt24{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt25{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt26{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt27{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt28{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt29{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt30{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt31{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt32{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt33{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt34{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt35{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt36{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt37{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt38{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt39{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt40{ type=\"xxxx\"; h=\"xxxx\";}\t\n";
	m_str << "\tt41{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt42{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt43{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt44{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt45{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt46{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt47{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt48{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt49{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\tt50{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "}\n";
	m_str << "\n";
	m_str << "\n";
	m_str << "# A safe for safety specifications\n";
	m_str << "# -----------------------------------------\n";
	m_str << "# For type = \"rectangle\", specify the per-dimensio";
	m_str << "n bounds in (h) as a comma-sepaarated list.\n";
	m_str << "safe{ \n";
	m_str << "\tcount=\"0\"; \n";
	m_str << "\ts1{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts2{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts3{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts4{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts5{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts6{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts7{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts8{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts9{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts10{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts11{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts12{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts13{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts14{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts15{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts16{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts17{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts18{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts19{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts20{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts21{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts22{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts23{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts24{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts25{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts26{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts27{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts28{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts29{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts30{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts31{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts32{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts33{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts34{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts35{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts36{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts37{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts38{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts19{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts40{ type=\"xxxx\"; h=\"xxxx\";}\t\n";
	m_str << "\ts41{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts42{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts43{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts44{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts45{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts46{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts47{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts48{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts49{ type=\"xxxx\"; h=\"xxxx\";}\n";
	m_str << "\ts50{ type=\"xxxx\"; h=\"xxxx\";}\t\n";
	m_str << "}\n";
	m_str << "\n";
	m_str << "";

	m_defaults = m_str.str();
}
defaultConfiguration::~defaultConfiguration(){
	// Nothing to do
}
const char* defaultConfiguration::getDefaults() {
	return s_singleton.m_defaults.c_str();
}
void defaultConfiguration::getSchema(const char**& schema, int& schemaSize) {
	schema = s_singleton.m_schema;
	schemaSize = 822;
}
const char** defaultConfiguration::getSchema(){
	return s_singleton.m_schema;
}


// ----------------
// The reader class
// ----------------
configReader::configReader(const std::shared_ptr<pfacesConfigurationReader>& spConfigObject) {
	m_spConfigObject = spConfigObject;
	m_spConfigObject->parse(defaultConfiguration::getSchema(), defaultConfiguration::getDefaults());
	
	// Extract config file directory for relative path resolution
	std::string config_path = m_spConfigObject->getConfigFilePath();
	std::cout << "[MonoSynth] Config file path: " << config_path << std::endl;
	size_t last_slash = config_path.find_last_of("/\\");
	if (last_slash != std::string::npos) {
		m_config_file_dir = config_path.substr(0, last_slash + 1);
	} else {
		m_config_file_dir = "./";
	}
	std::cout << "[MonoSynth] Config file dir: " << m_config_file_dir << std::endl;

	load_values();
	int validateResult = validate_values();
	if (validateResult == VALIDATE_RESULT_FAILED) {
		throw std::runtime_error(getValidationMessage());
	}
	if (validateResult == VALIDATE_RESULT_WARNING) {
		pfacesTerminal::showWarnMessage(getValidationMessage());
	}
}

std::string	configReader::getExtraIncludeFile()const{
	if (m_extraIncludeFile  == "xxxx")
		return "";
		
	return  m_extraIncludeFile;
}

void configReader::load_values() {
	//--------
		// Cache configuration variables in instance variables for 
		// faster access.
		//--------
	try {


		m_project_name = m_spConfigObject->readConfigValueString("project_name");


		try {
			std::string packSize = m_spConfigObject->readConfigValueString("synthesis_pack");
			size_t percPos = packSize.find("%");

			if (std::string("off") == packSize) {
				m_isSynthesisPackSizeExactorPerc = true;
				m_syntheisPackSize = 1;
			}
			else if (percPos != std::string::npos) {
				packSize.replace(percPos, 1, "");
				m_isSynthesisPackSizeExactorPerc = false;
				m_syntheisPackSize = std::atoi(packSize.c_str());
			}
			else {
				m_isSynthesisPackSizeExactorPerc = true;
				m_syntheisPackSize = std::atoi(packSize.c_str());
			}

		}
		catch (...) {
			m_isSynthesisPackSizeExactorPerc = true;
			m_syntheisPackSize = 1;
		}


		m_data = m_spConfigObject->readConfigValueString("data");
		m_save_transitions = m_spConfigObject->readConfigValueBool("save_transitions");
		m_save_controller = m_spConfigObject->readConfigValueBool("save_controller");

		try {
			m_record_basis_evolution = m_spConfigObject->readConfigValueBool("record_basis_evolution");
		} catch (...) {
			m_record_basis_evolution = false;
		}

		m_legacy_solver_key.clear();
		const char* legacy_keys[] = {
			"use_threshold_table", "use_tt_only", "use_tt_only_gpu",
			"use_bitmap_gfp", "use_inline_dynamics", "use_prefix_sweep",
			"boundary_seeding"
		};
		for (const char* key : legacy_keys) {
			const std::string value =
				m_spConfigObject->readConfigValueString(key);
			if (value != "__mono_synth_legacy_unset__" &&
			    m_legacy_solver_key.empty())
				m_legacy_solver_key = key;
		}

		std::string method = "threshold";
		try { method = m_spConfigObject->readConfigValueString("synthesis_method"); }
		catch (...) {}
		if (method == "cdc") m_synthesis_method = SynthesisMethod::CDC;
		else if (method == "automatica_scan") m_synthesis_method = SynthesisMethod::AUTOMATICA_SCAN;
		else if (method == "automatica_threshold") m_synthesis_method = SynthesisMethod::AUTOMATICA_THRESHOLD;
		else if (method == "threshold") m_synthesis_method = SynthesisMethod::THRESHOLD;
		else if (method == "bitmap_reference") m_synthesis_method = SynthesisMethod::BITMAP_REFERENCE;
		else if (method == "threshold_cpu_reference") m_synthesis_method = SynthesisMethod::THRESHOLD_CPU_REFERENCE;
		else throw pfacesConfigurationException(
			"synthesis_method must be cdc, automatica_scan, automatica_threshold, "
			"threshold, bitmap_reference, or threshold_cpu_reference");

		try {
			m_transition_semantics = m_spConfigObject->readConfigValueString("transition_semantics");
		} catch (...) {
			m_transition_semantics = "extremal_single_successor";
		}

		std::string backend = "precomputed";
		try { backend = m_spConfigObject->readConfigValueString("transition_backend"); }
		catch (...) {}
		if (backend == "precomputed") m_transition_backend = TransitionBackend::PRECOMPUTED;
		else if (backend == "inline") m_transition_backend = TransitionBackend::INLINE;
		else throw pfacesConfigurationException(
			"transition_backend must be precomputed or inline");

		std::string boundary_semantics = "strict_unsafe";
		try {
			boundary_semantics =
				m_spConfigObject->readConfigValueString("boundary_semantics");
		} catch (...) {}
		if (boundary_semantics == "strict_unsafe")
			m_boundary_semantics = BoundarySemantics::STRICT_UNSAFE;
		else if (boundary_semantics == "favorable_saturating")
			m_boundary_semantics = BoundarySemantics::FAVORABLE_SATURATING;
		else throw pfacesConfigurationException(
			"boundary_semantics must be strict_unsafe or favorable_saturating");

		try {
			m_extract_basis = m_spConfigObject->readConfigValueBool("extract_basis");
		} catch (...) {
			m_extract_basis = true;
		}

		try {
			m_threshold_d_star = m_spConfigObject->readConfigValueInt("threshold_d_star");
		} catch (...) {
			// -1 means "auto-select d* as widest dimension" (legacy behavior)
			m_threshold_d_star = -1;
		}

		m_statedim = m_spConfigObject->readConfigValueInt("states.dim");
		m_stateeta = m_spConfigObject->readConfigValueString("states.eta");
		m_statelb = m_spConfigObject->readConfigValueString("states.lb");
		m_stateub = m_spConfigObject->readConfigValueString("states.ub");
		m_stateerr = m_spConfigObject->readConfigValueString("states.err");

		m_inputdim = m_spConfigObject->readConfigValueInt("inputs.dim");
		m_inputeta = m_spConfigObject->readConfigValueString("inputs.eta");
		m_inputlb = m_spConfigObject->readConfigValueString("inputs.lb");
		m_inputub = m_spConfigObject->readConfigValueString("inputs.ub");
		m_inputerr = m_spConfigObject->readConfigValueString("inputs.err");

		m_sampling_period = m_spConfigObject->readConfigValueReal("sampling_period");
		
		// Read new mono_synth specific parameters
		try {
			m_ode_steps = m_spConfigObject->readConfigValueInt("ode_steps");
		} catch (...) {
			m_ode_steps = 1000; // Default
		}
		
		try {
			m_user_dynamics_file = m_spConfigObject->readConfigValueString("user_dynamics_file");
			std::cout << "[MonoSynth] Raw user dynamics file: " << m_user_dynamics_file << std::endl;
			// Make path absolute if it's relative
			if (!m_user_dynamics_file.empty() && m_user_dynamics_file[0] != '/') {
				m_user_dynamics_file = m_config_file_dir + m_user_dynamics_file;
			}
			std::cout << "[MonoSynth] Resolved user dynamics file: " << m_user_dynamics_file << std::endl;
		} catch (...) {
			m_user_dynamics_file = "";
		}
		
		try {
			m_state_priorities = m_spConfigObject->readConfigValueString("states.priorities");
			// Parse priorities manually (avoid template instantiation issues)
			std::vector<concrete_t> temp = pfacesUtils::sStr2Vector<concrete_t>(m_state_priorities);
			vSsPriorities.clear();
			for (auto val : temp) {
				vSsPriorities.push_back((int)val);
			}
		} catch (...) {
			// Default: all min-priority (increasing)
			vSsPriorities.resize(m_statedim, 1);
		}
		
		try {
			m_disturbdim = m_spConfigObject->readConfigValueInt("disturbances.dim");
		} catch (...) {
			m_disturbdim = 1; // Default
		}
		
		const int max_basis_elements =
			m_spConfigObject->readConfigValueInt("max_basis_elements");
		if (max_basis_elements <= 0)
			throw pfacesConfigurationException("max_basis_elements must be positive");
		m_max_basis_elements = static_cast<size_t>(max_basis_elements);

		try {
			m_benchmark_count = m_spConfigObject->readConfigValueInt("benchmark_count");
		} catch (...) {
			m_benchmark_count = 10;
		}

		// reading the extra include file
		m_extraIncludeFile = m_spConfigObject->readConfigValueString("extra_include_file");


		// reading reach sets
		try {
			m_targetCount = m_spConfigObject->readConfigValueInt("target.count");
			m_hasTarget = (m_targetCount > 0);
			std::string data = "";
			for (size_t i = 0; i < m_targetCount; i++) {
				std::string t_id = "target.t" + std::to_string(i + 1) + ".h";
				data += "{";
				data += pfacesUtils::strReplaceAll(m_spConfigObject->readConfigValueString(t_id.c_str()),std::string("_"),std::string(","));
				data += "}";
				if (i < m_targetCount - 1)
					data += ",";
			}
			m_targetData = data;
		}
		catch (const std::exception&) {
			m_hasTarget = false;
			m_targetCount = 0;
			m_targetData = "";
		}


		// reading safe sets
		try {
			m_safeCount = m_spConfigObject->readConfigValueInt("safe.count");
			m_hasSafe = (m_safeCount > 0);
			std::string data = "";
			for (size_t i = 0; i < m_safeCount; i++) {
				std::string t_id = "safe.s" + std::to_string(i + 1) + ".h";
				data += "{";
				data += pfacesUtils::strReplaceAll(m_spConfigObject->readConfigValueString(t_id.c_str()),std::string("_"),std::string(","));
				data += "}";
				if (i < m_safeCount - 1)
					data += ",";
			}
			m_safeData = data;
		}
		catch (const std::exception&) {
			m_hasSafe = false;
			m_safeCount = 0;
			m_safeData = "";
		}


		// reading avoid sets
		try {
			m_avoidCount = m_spConfigObject->readConfigValueInt("obstacles.count");
			m_hasAvoid = (m_avoidCount > 0);
			std::string data = "";
			for (size_t i = 0; i < m_avoidCount; i++) {
				std::string t_id = "obstacles.ob" + std::to_string(i + 1) + ".h";
				data += "{";
				data += pfacesUtils::strReplaceAll(m_spConfigObject->readConfigValueString(t_id.c_str()),std::string("_"),std::string(","));
				data += "}";
				if (i < m_avoidCount - 1)
					data += ",";
			}
			m_avoidData = data;
		}
		catch (const std::exception&) {
			m_hasAvoid = false;
			m_avoidCount = 0;
			m_avoidData = "";
		}

	}
	catch (std::exception ex) {
		throw pfacesConfigurationException(ex.what());
	}
}

int configReader::validate_values() {
	int ret = VALIDATE_RESULT_PASSED;
	std::stringstream sserrors("");
	std::stringstream sswarns("");

	sserrors << std::endl;
	sswarns << std::endl;

	std::string project_name(m_project_name);
	if (project_name.length() == 0) {
		sserrors << "\t-Project name should be provided." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}

	std::string data(m_data);
	if (!(data == "raw" || data == "bits" || data == "bdd")) {
		sserrors << "\t-Data should be: raw, bitset or bdd." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}

	if (!m_legacy_solver_key.empty()) {
		sserrors << "\t-Legacy solver key '" << m_legacy_solver_key
		         << "' is not supported. Replace all legacy solver booleans with "
		            "synthesis_method and transition_backend." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}
	if (m_transition_semantics != "extremal_single_successor") {
		sserrors << "\t-transition_semantics must be extremal_single_successor in this revision." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}
	const bool supports_inline =
		m_synthesis_method == SynthesisMethod::THRESHOLD ||
		m_synthesis_method == SynthesisMethod::BITMAP_REFERENCE;
	if (m_transition_backend == TransitionBackend::INLINE && !supports_inline) {
		sserrors << "\t-transition_backend=inline is allowed only for threshold and bitmap_reference." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}

	// sampling and ode usage validation
	if (m_sampling_period <= 0) {
		sserrors << "\t-Please provide non-negative sampling period." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}

	// State space validation
	if (m_statedim == 0) {
		sserrors << "\t-State space dimension can't be 0." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}
	else {
		vSsEta = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_stateeta));
		vSsLb = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_statelb));
		vSsUb = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_stateub));
		vSsErr = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_stateerr));

		if (vSsEta.size() != m_statedim || vSsLb.size() != m_statedim || vSsUb.size() != m_statedim || vSsErr.size() != m_statedim) {
			sserrors << "\t-Invalid number of elements in one or more of the state space parameters." << std::endl;
			ret = VALIDATE_RESULT_FAILED;
		}

		if (m_threshold_d_star < -1 || m_threshold_d_star >= (int)m_statedim) {
			sserrors << "\t-threshold_d_star must be -1 (auto) or in [0, states.dim-1]." << std::endl;
			ret = VALIDATE_RESULT_FAILED;
		}
	}

	// Input space validation
	if (m_inputdim == 0) {
		sserrors << "\t-Input space dimension can't be 0." << std::endl;
		ret = VALIDATE_RESULT_FAILED;
	}
	else {
		vIsEta = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_inputeta));
		vIsLb = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_inputlb));
		vIsUb = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_inputub));
		vIsErr = pfacesUtils::sStr2Vector<concrete_t>(std::string(m_inputerr));

		if (vIsEta.size() != m_inputdim || vIsLb.size() != m_inputdim || vIsUb.size() != m_inputdim || vIsErr.size() != m_inputdim) {
			sserrors << "\t-Invalid number of elements in one or more of the input space parameters." << std::endl;
			ret = VALIDATE_RESULT_FAILED;
		}
	}

	// Check final validation result and set message
	if (ret == VALIDATE_RESULT_FAILED) {
		m_validatemsg = (sserrors.str()).c_str();
		return ret;
	}


	// Warnings

	return ret;
}
