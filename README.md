`timescale 1ns/1ps

module BATCHARGER_64b_tb;

   // -----------------------
   // Digital control signals
   // -----------------------
   reg        clk;
   reg        rstz;
   reg        en;
   reg  [3:0] sel;

   // -----------------------
   // Analog 64-bit I/O
   // -----------------------
   wire [63:0] iforcedbat;
   wire [63:0] vsensbat;
   wire [63:0] vin;
   wire [63:0] vbattemp;
   wire [63:0] dvdd;
   wire [63:0] dgnd;
   wire [63:0] pgnd;

   // -----------------------
   // Real-valued mirrors
   // -----------------------
   real rl_iforcedbat;
   real rl_vsensbat;
   real rl_vin;
   real rl_vbattemp;
   real rl_dvdd;
   real rl_dgnd;
   real rl_pgnd;

   // -----------------------
   // Expected current variables
   // -----------------------
   real expected_current;
   real Vtarget;
   real R;

   parameter real tolerance = 1.0e-3;

   // -----------------------
   // Read internal DAC codes from TOP
   // -----------------------
   wire [7:0] icc = dut.icc;
   wire [7:0] itc = dut.itc;
   wire [7:0] vcv = dut.vcv;

   // -----------------------
   // Real <-> 64-bit mapping
   // -----------------------
   assign vsensbat = $realtobits(rl_vsensbat);
   assign vin      = $realtobits(rl_vin);
   assign vbattemp = $realtobits(rl_vbattemp);
   assign dvdd     = $realtobits(rl_dvdd);
   assign dgnd     = $realtobits(rl_dgnd);
   assign pgnd     = $realtobits(rl_pgnd);

   always @(*) rl_iforcedbat = $bitstoreal(iforcedbat);

   // -----------------------
   // DUT instantiation (TOP MODULE)
   // -----------------------
   BATCHARGER_64b dut (
      .iforcedbat(iforcedbat),
      .vsensbat(vsensbat),
      .vin(vin),
      .vbattemp(vbattemp),
      .en(en),
      .sel(sel),
      .clk(clk),
      .rstz(rstz),
      .dvdd(dvdd),
      .dgnd(dgnd),
      .pgnd(pgnd)
   );

   // -----------------------
   // Clock generation
   // -----------------------
   initial clk = 0;
   always #5 clk = ~clk;

   // ============================================================
   // TEST SEQUENCE — ONLY POWER MODE CURRENT MATCH TESTS
   // ============================================================
   initial begin

      // Basic supplies
      rl_dvdd = 1.8;
      rl_dgnd = 0.0;
      rl_pgnd = 0.0;

      // Battery & VIN
      rl_vin      = 5.0;
      rl_vsensbat = 3.2;
      rl_vbattemp = 0.5;   // any mid-range temperature

      sel = 4'b0111;  // C = 400 mAh
      en  = 0;
      rstz = 0;

      #100;
      en = 1;
      rstz = 1;

      #500; // stabilization

      $display("\n\n========== POWER BLOCK CURRENT TESTS (TOP MODULE) ==========");
      $display("Battery size C = %0.3f A", calc_C(sel));

      // ----------------------------------------------------------
      // TEST 1 : TRICKLE MODE (TC)
      // ----------------------------------------------------------
      $display("\n--- TEST 1: Trickle Mode (TC) ---");

      force dut.tc = 1;
      force dut.cc = 0;
      force dut.cv = 0;

      rl_vsensbat = 3.0; // low Vbat

      #200;

      expected_current =
         calc_C(sel) *
        (0.502 * itc[7] + 0.251 * itc[6] + 0.1255 * itc[5] + 0.0627 * itc[4] +
         0.0314 * itc[3] + 0.0157 * itc[2] + 0.0078 * itc[1] + 0.0039 * itc[0]);

      compare_result("Trickle", expected_current, rl_iforcedbat);

      // ----------------------------------------------------------
      // TEST 2 : CONSTANT CURRENT MODE (CC)
      // ----------------------------------------------------------
      $display("\n--- TEST 2: Constant Current Mode (CC) ---");

      force dut.tc = 0;
      force dut.cc = 1;
      force dut.cv = 0;

      rl_vsensbat = 3.7;

      #200;

      expected_current =
         calc_C(sel) *
        (0.502 * icc[7] + 0.251 * icc[6] + 0.1255 * icc[5] + 0.0627 * icc[4] +
         0.0314 * icc[3] + 0.0157 * icc[2] + 0.0078 * icc[1] + 0.0039 * icc[0]);

      compare_result("Constant Current", expected_current, rl_iforcedbat);

      // ----------------------------------------------------------
      // TEST 3 : CONSTANT VOLTAGE MODE (CV)
      // ----------------------------------------------------------
      $display("\n--- TEST 3: Constant Voltage Mode (CV) ---");

      force dut.tc = 0;
      force dut.cc = 0;
      force dut.cv = 1;

      // compute resistor
      R = 0.4 / (0.5 * calc_C(sel));

      // compute target voltage using DAC
      Vtarget = 5.0 *
         (0.502 * vcv[7] + 0.251 * vcv[6] + 0.1255 * vcv[5] + 0.0627 * vcv[4] +
          0.0314 * vcv[3] + 0.0157 * vcv[2] + 0.0078 * vcv[1] + 0.0039 * vcv[0]);

      // --- Case A : below target ---
      rl_vsensbat = Vtarget - 0.2;
      #200;
      expected_current = (Vtarget - rl_vsensbat) / R;
      compare_result("CV below target", expected_current, rl_iforcedbat);

      // --- Case B : near target ---
      rl_vsensbat = Vtarget - 0.01;
      #200;
      expected_current = (Vtarget - rl_vsensbat) / R;
      compare_result("CV near target", expected_current, rl_iforcedbat);

      // --- Case C : Vsensbat > Vtarget → current = 0 ---
      rl_vsensbat = Vtarget + 0.05;
      #200;
      expected_current = (Vtarget - rl_vsensbat) / R; 
      compare_result("CV above target (should be 0)", expected_current, rl_iforcedbat);

      // release forced signals
      release dut.tc;
      release dut.cc;
      release dut.cv;

      $display("\n========== ALL POWER-BLOCK CURRENT TESTS PASSED ==========\n");
      $finish;

   end

   // ============================================================
   // Supporting TASKS
   // ============================================================

   task compare_result;
      input [31*8:1] operation_mode;
      input real expected_value;
      input real actual_value;

      real difference;
   begin
      difference = (expected_value > actual_value) ?
                   (expected_value - actual_value) :
                   (actual_value - expected_value);

      if (difference < tolerance) begin
         $display("PASS [%s] Expected=%f Actual=%f Δ=%f",
                  operation_mode, expected_value, actual_value, difference);
      end else begin
         $display("FAIL [%s] Expected=%f Actual=%f Δ=%f",
                  operation_mode, expected_value, actual_value, difference);
         $finish;
      end
   end
   endtask


   function real calc_C(input [3:0] sel);
      real C;
      begin
         C = 50.0e-3;
         if (sel[0]) C = C +  50.0e-3;
         if (sel[1]) C = C + 100.0e-3;
         if (sel[2]) C = C + 200.0e-3;
         if (sel[3]) C = C + 400.0e-3;
         calc_C = C;
      end
   endfunction

endmodule






*************************************
`timescale 1ns / 1ps

module BATCHARGERpowerfixed_64_tb;

   // --------- PARAMETERS FROM DATASHEET (for sanity_check_power) ---------
   localparam real VREFMIN  = 0.45;
   localparam real VREFMAX  = 0.55;
   localparam real IBIASMIN = 0.9e-6;
   localparam real IBIASMAX = 1.1e-6;
   localparam real VINMIN   = 3.0;
   localparam real DROPMIN  = 0.2;

   parameter tolerance = 1.0e-3; // Tolerância de 1mA, por exemplo

   wire [63:0] iforcedbat; // output current to battery
   wire [63:0] vbatcurr;   // ibat value scaled 1000:1 * (R=Vref/C)
   wire [63:0] vsensbat;   // voltage sensed
   wire [63:0] vref;       // voltage reference (vref = 0.5V)
   wire [63:0] vin;        // input voltage; must be at least 200mV higher than vsensbat
   wire [63:0] ibias1u;    // reference current (ibias1u = 1uA)

   reg  [7:0]  icc;
   reg  [7:0]  itc;
   reg  [7:0]  vcv;
   reg         cc;
   reg         tc;
   reg         cv;
   reg         en;
   reg  [3:0]  sel;

   wire [63:0] dvdd;  // digital supply
   wire [63:0] dgnd;  // digital ground
   wire [63:0] avdd;  // analog supply
   wire [63:0] agnd;  // analog ground
   wire [63:0] pgnd;  // power ground

   real rl_avdd;
   real rl_dvdd;
   real rl_agnd;
   real rl_dgnd;
   real rl_pgnd;     

   real R;

   real rl_vbatcurr;   // vbatcurr real value 
   real rl_vref;       // converted value of vref to real
   real rl_iforcedbat; // iforcedbat real value
   real rl_vsensbat;   // converted value of vsensbat to real 
   real rl_vin;        // converted value of vin to real 
   real rl_ibias1u;    // converted value of ibias1u to real 

   real rl_icc;
   real rl_itc;
   real rl_vcv;

   // Variaveis auxiliares para utilizar no Test Bench Inteligente
   //real rl_C;              // Valor de C em Amperes
   real expected_current;  // Valor esperado da corrente de carga
   real Vtarget;
   reg  supply_ok;

   // DUT -------------------------------------------------------------------
   BATCHARGERpower_64b uut (
      .iforcedbat(iforcedbat),
      .vbatcurr  (vbatcurr),
      .vsensbat  (vsensbat),
      .vref      (vref),
      .vin       (vin),
      .ibias1u   (ibias1u),
      .icc       (icc),
      .itc       (itc),
      .vcv       (vcv),
      .cc        (cc),
      .tc        (tc),
      .cv        (cv),
      .en        (en),
      .sel       (sel),
      .avdd      (avdd),
      .dvdd      (dvdd),
      .dgnd      (dgnd),
      .agnd      (agnd),
      .pgnd      (pgnd)
   ); 


   initial begin
    
    $display("---Sanity Checks---");
    
      // ============================
      // 1) VREF too LOW (<0.45V)
      // ============================
      en = 1;
      tc = 0; cc = 0; cv = 0;

      rl_vin      = 5.0;     // OK
      rl_vsensbat = 3.7;     // OK
      rl_vref     = 0.40;    // too low
      rl_ibias1u  = 1.0e-6;  // OK

      #10;
      sanity_check_power("SANITY: VREF too LOW",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("VREF too LOW", expected_current, rl_iforcedbat);

      // ============================
      // 2) VREF too HIGH (>0.55V)
      // ============================
      rl_vref     = 0.60;    // too high

      #10;
      sanity_check_power("SANITY: VREF too HIGH",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("VREF too HIGH", expected_current, rl_iforcedbat);
      // ============================
      // 3) IBIAS too LOW (<0.9uA)
      // ============================
      rl_vref     = 0.50;    // back to OK
      rl_ibias1u  = 0.5e-6;  // too low

      #10;
      sanity_check_power("SANITY: IBIAS too LOW",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("IBIAS too LOW", expected_current, rl_iforcedbat);
      // ============================
      // 4) IBIAS too HIGH (>1.1uA)
      // ============================
      rl_ibias1u  = 2.0e-6;  // too high

      #10;
      sanity_check_power("SANITY: IBIAS too HIGH",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("IBIAS too HIGH", expected_current, rl_iforcedbat);
      // ============================
      // 5) VIN below VINMIN (<3.0V)
      // ============================
      rl_ibias1u  = 1.0e-6;  // back to OK
      rl_vin      = 2.5;     // too low

      #10;
      sanity_check_power("SANITY: VIN below VINMIN",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("VIN below VINMIN", expected_current, rl_iforcedbat);
      // ============================
      // 6) VIN OK but DROPOUT violated
      // ============================
      rl_vin      = 3.8; // moderate VIN
      rl_vsensbat = 3.7; // OK dropout (3.8 >= 3.7 + 0.2)

      #10;
      sanity_check_power("SANITY: DROPOUT OK (reference)",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("DROPOUT OK (reference)", expected_current, rl_iforcedbat);
      // Now violate dropout:
      rl_vsensbat = 3.7;
      rl_vin      = 3.8;  // needs >= 3.9 → violation

      #10;
      sanity_check_power("SANITY: DROPOUT violated",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("DROPOUT violated", expected_current, rl_iforcedbat);
      // ============================
      // 7) EN = 0 (charger disabled)
      // ============================
      rl_vin      = 5.0;
      rl_vsensbat = 3.7;
      rl_vref     = 0.50;
      rl_ibias1u  = 1.0e-6;

      en          = 0;   // disabled

      #10;
      sanity_check_power("SANITY: EN=0 (disabled)",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = 0.0;
      compare_result("SANITY: EN=0 (disabled)", expected_current, rl_iforcedbat);
      // start of TC/CC/CV mode tests.
      en = 1;
      sel[3:0]    = 4'b0111;  // C=400mAh     
      rl_vsensbat = 3.2; 
      rl_vref     = 0.5;
      rl_vin      = 5.0; 
      rl_ibias1u  = 1.0e-6; 
      icc[7:0]    = 8'b01111111; 
      itc[7:0]    = 8'b00011001; 
      vcv[7:0]    = 8'b10111100; // 3.7 V

      $display("---OPERATION MODES---");
      $display("Configuration: C = %0.3f A, Vbat = %0.2f V",calc_C(sel), rl_vsensbat);
      #100; // Aguarda estabilização inicial

      //--------- TESTE 1 : Trickle Mode --------- 
      cv = 1'b0;
      cc = 1'b0;
      tc = 1'b1;
      #100;
      sanity_check_power("Trickle (TC)",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = calc_C(sel) *
          (0.502 * itc[7] + 0.251 * itc[6] + 0.1255 * itc[5] + 0.0627 * itc[4] +
           0.0314 * itc[3] + 0.0157 * itc[2] + 0.0078 * itc[1] + 0.0039 * itc[0]);

      compare_result("Trickle (TC)", expected_current, rl_iforcedbat);

      //--------- TESTE 2 : Constant Current Mode --------- 
      cv = 1'b0;
      cc = 1'b1;
      tc = 1'b0;
      #100;
      sanity_check_power("Constant Current (CC)",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      expected_current = calc_C(sel) *
          (0.502 * icc[7] + 0.251 * icc[6] + 0.1255 * icc[5] + 0.0627 * icc[4] +
           0.0314 * icc[3] + 0.0157 * icc[2] + 0.0078 * icc[1] + 0.0039 * icc[0]);

      compare_result("Constant Current (CC)", expected_current, rl_iforcedbat);

      //--------- TESTE 3 : Constant Voltage Mode ---------
      cv = 1'b1;
      cc = 1'b0;
      tc = 1'b0;
      #100;
      sanity_check_power("Constant Voltage (CV)",
                         rl_vin, rl_vsensbat, rl_vref, rl_ibias1u,
                         en, tc, cc, cv,
                         supply_ok);
      R = 0.4 / (0.5 * calc_C(sel));
      Vtarget = 5.0 *
          (0.502 * vcv[7] + 0.251 * vcv[6] + 0.1255 * vcv[5] + 0.0627 * vcv[4] +
           0.0314 * vcv[3] + 0.0157 * vcv[2] + 0.0078 * vcv[1] + 0.0039 * vcv[0]);

      expected_current = (Vtarget - rl_vsensbat) / R;

      compare_result("Constant Voltage (CV)", expected_current, rl_iforcedbat);

      $display("--- TODOS OS TESTES PASSARAM ---");
      $finish;
   end

   //-- Signal conversion ------------------
   assign vref     = $realtobits (rl_vref);
   assign vsensbat = $realtobits (rl_vsensbat);
   assign vin      = $realtobits (rl_vin);
   assign ibias1u  = $realtobits (rl_ibias1u);
   assign pgnd     = $realtobits (rl_pgnd);
   assign avdd     = $realtobits (rl_avdd);
   assign dvdd     = $realtobits (rl_dvdd);
   assign agnd     = $realtobits (rl_agnd);
   assign dgnd     = $realtobits (rl_dgnd);  

   initial assign rl_iforcedbat = $bitstoreal (iforcedbat);
   initial assign rl_vbatcurr   = $bitstoreal (vbatcurr);   

   // -------------------- TASKS --------------------------

   task sanity_check_power;
      input  [31*8:1] op_name;     // string describing the test
      input  real rl_vin;
      input  real rl_vsensbat;
      input  real rl_vref;
      input  real rl_ibias1u;
      input        en, tc, cc, cv;
      output       supply_ok;

      reg vref_ok, ibias_ok, vin_ok;
   begin
      vref_ok  = (rl_vref    >= VREFMIN  && rl_vref    <= VREFMAX);
      ibias_ok = (rl_ibias1u >= IBIASMIN && rl_ibias1u <= IBIASMAX);

      vin_ok   = (rl_vin >= VINMIN) &&
                 (rl_vin >= rl_vsensbat + DROPMIN);

      supply_ok = (en && vref_ok && ibias_ok && vin_ok);

      if (!vref_ok)
         $display("SANITY ERROR [%s]: VREF out of range (%.3f V). Allowed: %.3f–%.3f V",
                  op_name, rl_vref, VREFMIN, VREFMAX);

      if (!ibias_ok)
         $display("SANITY ERROR [%s]: IBIAS out of range (%.3e A). Allowed: %.3e–%.3e A",
                  op_name, rl_ibias1u, IBIASMIN, IBIASMAX);

      if (!vin_ok)
         $display("SANITY ERROR [%s]: VIN invalid (VIN=%.3f V, Vbat=%.3f V). Needs VIN>=%.1f V AND VIN>=Vbat+%.1f V",
                  op_name, rl_vin, rl_vsensbat, VINMIN, DROPMIN);

      if (!en)
         $display("SANITY ERROR [%s]: EN=0 → charger disabled.", op_name);

      if (!supply_ok)
         $display("supply_ok = 0 → Expected current MUST be 0");
      else 
         $display("supply_ok = 1 → Charger can operate normally");
   end
   endtask

   task compare_result;
      input [31*8:1] operation_mode;  // string with mode name
      input real expected_value;
      input real actual_value;
      real difference;
   begin
      difference = (expected_value > actual_value) ? 
                   (expected_value - actual_value) : 
                   (actual_value - expected_value);

      if (difference < tolerance) begin
         $display("Current Check - TEST PASSED: %s | Expected: %f | Actual: %f | Δ = %f", 
                  operation_mode, expected_value, actual_value, difference);
      end 
      else begin
         $display("CURRENT CHECK - TEST FAILED: %s | Expected: %f | Actual: %f | Δ = %f", 
                  operation_mode, expected_value, actual_value, difference);
         $finish;
      end
   end
   endtask

   function real calc_C(input [3:0] sel);
    real C;
    begin
        C = 50.0e-3; 
        if (sel[0]) C = C +  50.0e-3;
        if (sel[1]) C = C + 100.0e-3;
        if (sel[2]) C = C + 200.0e-3;
        if (sel[3]) C = C + 400.0e-3;
        calc_C = C;
    end
endfunction


endmodule
**************

module BATCHARGERpower_64b (
       output [63:0] iforcedbat, // output current to battery
       output [63:0] vbatcurr, // scaled mirrored current of iforcedbat (1000:1) x R ; R=Vref/C  (C is battery capacity)
       input [63:0]  vsensbat, // voltage sensed (obtained at the battery as "voltage from iforcedbat integration" + ESR * iforcedbat )			    
       input [63:0]  vref, // voltage reference (vref = 0.5V)
       input [63:0]  vin, // input voltage; must be at least 200mV higher than vsensbat to allow iforcedbat > 0
       input [63:0]  ibias1u, // reference current	(ibias1u = 1uA)
       input [7:0]   icc, // constant current mode output current value icc=8'b1111_1111 -> iforced = 2A; ex: icc=8'b11011111 -> iforced = 1.75A (0.5C)
       input [7:0]   itc, //  trickle current mode output current value itc=8'b1111_1111 -> iforced = 2A; ex: itc=8'b00101100 -> iforced = 0.35A (0.1C)
       input [7:0]   vcv, // constant voltage target value vcv = Vtarget*255/5 = 51*Vtarget
       input 	     cc, // enables constant current charging mode
       input 	     tc, // enables trickle  current charging mode 
       input 	     cv, // enables constant voltage charging mode
       input 	     en, // enables the module
       input [3:0]   sel, // battery capacity selection bits: b[3,2,1,0] weights are 400,200,100,50 mAh + offset of 50mAh covers the range from 50 up to 800 mAh 
       inout [63:0]  avdd, // analog supply to other modules
       inout [63:0]  dvdd, // digital supply to other modules
       inout [63:0]  dgnd, // digital ground
       inout [63:0]  agnd, // analog ground
       inout [63:0]  pgnd // power ground
); // enables the module
			    

   parameter IBIASMAX = 1.1e-6;
   parameter IBIASMIN = 0.9e-6; // current limits for ibias1u acceptance
   parameter VINMIN = 3; // minimum vin
   parameter DROPMIN = 0.2; // minimum difference between vin and vsensbat
   parameter VREFMIN = 0.45;
   parameter VREFMAX = 0.55; // voltage limits for vref acceptance
   

  
   real rl_vref;         // converted value of vref to real
   real rl_imirr;      // scaled mirrored current of iforcedbat (1000:1)
   real rl_iforcedbat;         // iforcedbat real value
   real rl_vsensbat;         // converted value of vsensbat to real 
   real rl_vin;         // converted value of vin to real 
   real rl_ibias1u;         // converted value of ibias1u to real 
   real rl_resibat;
   real rl_vbatcurr;
   real rl_C;       // [Ah] battery capacity
   real rl_Rch;     // [Ohm] constant voltage mode resistence: zero current step if Rch = (vcv - vpreset) / icc; Ex (4.2-3.8)/0.21 = 1.9 Ohm
   
   
   real rl_icc;
   real rl_itc;
   real rl_vcv;
    
   wire vref_ok;
   wire ibias_ok;
   wire vin_ok;
   wire supply_ok;



always @* begin

  // ====================================================
  // 1) Compute battery capacity C (in mAh) from sel[3:0]
  //    Valid range: 50 mAh → 800 mAh
  // ====================================================
    // C in A (50 mA → 0.050, etc.)
  rl_C = 50.0e-3;                     // 50 mA → 0.050 A
  if (sel[0]) rl_C = rl_C +  50.0e-3; // +50  mA
  if (sel[1]) rl_C = rl_C + 100.0e-3; // +100 mA
  if (sel[2]) rl_C = rl_C + 200.0e-3; // +200 mA
  if (sel[3]) rl_C = rl_C + 400.0e-3; // +400 mA


  // ====================================================
  // 2) Default: no charging current
  // ====================================================
  rl_iforcedbat = 0.0;

  // ====================================================
  // 3) Mode selection (only active if supply_ok == 1)
  //    TC, CC, CV — mutually exclusive
  // ====================================================
  if (supply_ok) begin

    // -------------------------------
    // CC mode (Constant Current)
    // iforcedbat = C × fractional ICC
    // -------------------------------
    if (cc) begin
        rl_icc = (
            0.502  * icc[7] +
            0.251  * icc[6] +
            0.1255 * icc[5] +
            0.0627 * icc[4] +
            0.0314 * icc[3] +
            0.0157 * icc[2] +
            0.0078 * icc[1] +
            0.0039 * icc[0]
        );
        rl_iforcedbat = rl_C * rl_icc;
    end

    // -------------------------------
    // TC mode (Trickle Current)
    // iforcedbat = C × fractional ITC
    // -------------------------------
    else if (tc) begin
        rl_itc = (
            0.502  * itc[7] +
            0.251  * itc[6] +
            0.1255 * itc[5] +
            0.0627 * itc[4] +
            0.0314 * itc[3] +
            0.0157 * itc[2] +
            0.0078 * itc[1] +
            0.0039 * itc[0]
        );
        rl_iforcedbat = rl_C * rl_itc;
    end

    // -------------------------------
    // CV mode (Constant Voltage)
    // iforcedbat = (Vtarget − Vsense) / Rch
    // where:
    //   Vtarget derived from vcv
    //   Rch = 0.4 / (0.5 × C)
    // -------------------------------
    else if (cv) begin
        rl_vcv = 5 * (
            0.502  * vcv[7] +
            0.251  * vcv[6] +
            0.1255 * vcv[5] +
            0.0627 * vcv[4] +
            0.0314 * vcv[3] +
            0.0157 * vcv[2] +
            0.0078 * vcv[1] +
            0.0039 * vcv[0]
        );
        rl_Rch = 0.4 / (0.5 * rl_C);
        rl_iforcedbat = (rl_vcv - rl_vsensbat) / rl_Rch;
    end

    // -------------------------------
    // No mode selected → keep 0 A
    // -------------------------------
    else begin
        // nothing to do
    end
  end

  // ====================================================
  // 4) Output vbatcurr (scaled current mirror)
  //    vbatcurr = iforcedbat × (vref / C)
  // ====================================================
  rl_vbatcurr = rl_iforcedbat * (rl_vref / rl_C);

end


// Sannity Checks
assign vref_ok  = (rl_vref >= VREFMIN  && rl_vref <= VREFMAX);
assign ibias_ok = (rl_ibias1u >= IBIASMIN && rl_ibias1u <= IBIASMAX);
assign vin_ok   = (rl_vin >= rl_vsensbat + DROPMIN);
assign supply_ok = (en && vin_ok && vref_ok && ibias_ok);

// Signal Conversion
// 64-bit outputs (nets) ⇐ real
assign iforcedbat = $realtobits(rl_iforcedbat);
assign vbatcurr   = $realtobits(rl_vbatcurr);

// real mirrors (variables) ⇐ 64-bit inputs
always @* begin
    rl_vref     = $bitstoreal(vref);
    rl_vsensbat = $bitstoreal(vsensbat);
    rl_vin      = $bitstoreal(vin);
    rl_ibias1u  = $bitstoreal(ibias1u);
end

    
endmodule
***************

`timescale 1ns/1ps

module BATCHARGER_64b_tb;

   // -----------------------
   // Digital control signals
   // -----------------------
   reg        clk;
   reg        rstz;
   reg        en;
   reg  [3:0] sel;

   // -----------------------
   // 64-bit "analog" ports
   // -----------------------
   wire [63:0] iforcedbat;
   wire [63:0] vsensbat;
   wire [63:0] vin;
   wire [63:0] vbattemp;
   wire [63:0] dvdd;
   wire [63:0] dgnd;
   wire [63:0] pgnd;

   // -----------------------
   // Real-valued mirrors
   // -----------------------
   real rl_iforcedbat;
   real rl_vsensbat;
   real rl_vin;
   real rl_vbattemp;
   real rl_dvdd;
   real rl_dgnd;
   real rl_pgnd;

   // -----------------------
   // Parameters for current check
   // -----------------------
   real expected_current;
   real Vtarget;
   real R;
   parameter real tolerance = 1.0e-3; // 1 mA tolerance

   // aliased internal mode-code and DAC-code signals from DUT
   wire [7:0] icc;
   wire [7:0] itc;
   wire [7:0] vcv;

   assign icc = BATCHARGER_64b_tb.DUT.icc;
   assign itc = BATCHARGER_64b_tb.DUT.itc;
   assign vcv = BATCHARGER_64b_tb.DUT.vcv;

   // -----------------------
   // Real <-> 64-bit mapping
   // -----------------------
   assign vsensbat = $realtobits(rl_vsensbat);
   assign vin      = $realtobits(rl_vin);
   assign vbattemp = $realtobits(rl_vbattemp);
   assign dvdd     = $realtobits(rl_dvdd);
   assign dgnd     = $realtobits(rl_dgnd);
   assign pgnd     = $realtobits(rl_pgnd);

   // Read current from DUT (continuous)
   always @(*) begin
      rl_iforcedbat = $bitstoreal(iforcedbat);
   end

   // -----------------------
   // DUT instantiation
   // -----------------------
   BATCHARGER_64b DUT (
      .iforcedbat(iforcedbat),
      .vsensbat(vsensbat),
      .vin(vin),
      .vbattemp(vbattemp),
      .en(en),
      .sel(sel),
      .clk(clk),
      .rstz(rstz),
      .dvdd(dvdd),
      .dgnd(dgnd),
      .pgnd(pgnd)
   );

   // -----------------------
   // Clock generation
   // -----------------------
   initial clk = 1'b0;
   always #5 clk = ~clk;   // 100 MHz clock

   // -----------------------
   // MAIN STIMULUS: mode tests for power block
   // -----------------------
   initial begin
      // Basic supplies
      rl_dvdd    = 1.8;
      rl_dgnd    = 0.0;
      rl_pgnd    = 0.0;

      // VIN and battery
      rl_vin      = 5.0;
      rl_vsensbat = 3.2;     // some mid voltage
      rl_vbattemp = 0.5;     // temp in safe region (depends on SAR scaling)

      // Digital control
      en   = 1'b0;
      sel  = 4'b0111;        // same as your power TB: C = 400 mAh
      rstz = 1'b0;

      // Bring DUT out of reset and enable
      #100;
      rstz = 1'b1;
      en   = 1'b1;

      // Wait for BG / internal stuff to settle a bit
      #1000;

      $display("--- OPERATION MODES (POWER BLOCK VIA TOP) ---");
      $display("Configuration: C = %0.3f A, Vbat = %0.2f V",
               calc_C(sel), rl_vsensbat);

      // =================================================
      // TEST 1 : TRICKLE MODE (TC)
      // =================================================
      $display("\n--- TEST 1: Trickle (TC) ---");
      force DUT.tc = 1'b1;
      force DUT.cc = 1'b0;
      force DUT.cv = 1'b0;

      // choose a "discharged" battery voltage
      rl_vsensbat = 3.2;
      #200; // let analog settle

      expected_current = calc_C(sel) *
          (0.502 * itc[7] + 0.251 * itc[6] + 0.1255 * itc[5] + 0.0627 * itc[4] +
           0.0314 * itc[3] + 0.0157 * itc[2] + 0.0078 * itc[1] + 0.0039 * itc[0]);

      compare_result("Trickle (TC)", expected_current, rl_iforcedbat);

      // =================================================
      // TEST 2 : CONSTANT CURRENT MODE (CC)
      // =================================================
      $display("\n--- TEST 2: Constant Current (CC) ---");
      force DUT.tc = 1'b0;
      force DUT.cc = 1'b1;
      force DUT.cv = 1'b0;

      rl_vsensbat = 3.7; // still below Vtarget so CC should be active
      #200;

      expected_current = calc_C(sel) *
          (0.502 * icc[7] + 0.251 * icc[6] + 0.1255 * icc[5] + 0.0627 * icc[4] +
           0.0314 * icc[3] + 0.0157 * icc[2] + 0.0078 * icc[1] + 0.0039 * icc[0]);

      compare_result("Constant Current (CC)", expected_current, rl_iforcedbat);

      // =================================================
      // TEST 3 : CONSTANT VOLTAGE MODE (CV)
      // =================================================
      $display("\n--- TEST 3: Constant Voltage (CV) ---");
      force DUT.tc = 1'b0;
      force DUT.cc = 1'b0;
      force DUT.cv = 1'b1;

      // NOTE: vcv is inside DUT; we read it hierarchically.
      // R = 0.4 / (0.5 * C)
      R = 0.4 / (0.5 * calc_C(sel));

      // Vtarget from 8-bit DAC code vcv; 5.0 is full-scale (per your TB logic)
      Vtarget = 5.0 *
          (0.502  * vcv[7] + 0.251  * vcv[6] + 0.1255 * vcv[5] + 0.0627 * vcv[4] +
           0.0314 * vcv[3] + 0.0157 * vcv[2] + 0.0078 * vcv[1] + 0.0039 * vcv[0]);

      // --- Case: Vbat below Vtarget ---
      rl_vsensbat = Vtarget - 0.2;   // 200mV below target
      #200;
      expected_current = (Vtarget - rl_vsensbat) / R;
      compare_result("Constant Voltage (CV) - below Vtarget", expected_current, rl_iforcedbat);

      // --- Case: Vbat very close to Vtarget (small current) ---
      rl_vsensbat = Vtarget - 0.01;  // 10mV below
      #200;
      expected_current = (Vtarget - rl_vsensbat) / R;
      compare_result("Constant Voltage (CV) - near Vtarget", expected_current, rl_iforcedbat);

      // Release forced signals
      release DUT.tc;
      release DUT.cc;
      release DUT.cv;

      $display("\n--- ALL POWER-BLOCK CURRENT TESTS (TOP MODULE) PASSED ---");
      $finish;
   end

   task compare_result;
      input [31*8:1] operation_mode;  // string with mode name
      input real expected_value;
      input real actual_value;
      real difference;
   begin
      difference = (expected_value > actual_value) ? 
                   (expected_value - actual_value) : 
                   (actual_value - expected_value);

      if (difference < tolerance) begin
         $display("Current Check - TEST PASSED: %s | Expected: %f | Actual: %f | Δ = %f", 
                  operation_mode, expected_value, actual_value, difference);
      end 
      else begin
         $display("CURRENT CHECK - TEST FAILED: %s | Expected: %f | Actual: %f | Δ = %f", 
                  operation_mode, expected_value, actual_value, difference);
         $finish;
      end
   end
   endtask

   // -----------------------
   // calc_C FUNCTION
   // (same logic you used before)
   // -----------------------
   function real calc_C(input [3:0] sel);
      real C;
      begin
         C = 50.0e-3; 
         if (sel[0]) C = C +  50.0e-3;
         if (sel[1]) C = C + 100.0e-3;
         if (sel[2]) C = C + 200.0e-3;
         if (sel[3]) C = C + 400.0e-3;
         calc_C = C;
      end
   endfunction

endmodule
