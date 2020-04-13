library IEEE;
use IEEE.std_logic_1164.ALL;
use IEEE.std_logic_arith.ALL;
use IEEE.std_logic_unsigned.ALL;

use work.report_decoded_pack.all;

entity usbhid_report_decoder is
generic
(
  C_reg_input: boolean := true; -- take input in register (release timing)
  -- mouse speed also depends on clk
  C_lmouse: boolean := false;
  C_lmousex_scaler: integer := 23; -- less -> faster mouse
  C_lmousey_scaler: integer := 23; -- less -> faster mouse
  C_rmouse: boolean := false;
  C_rmousex_scaler: integer := 23; -- less -> faster mouse
  C_rmousey_scaler: integer := 23  -- less -> faster mouse
);
port
(
  clk: in std_logic; -- USB host core clock domain
  hid_report: in std_logic_vector;
  hid_valid: in std_logic;
  decoded: out T_report_decoded
);
end;

architecture rtl of usbhid_report_decoder is
  signal R_hid_report: std_logic_vector(hid_report'range);
  signal R_hid_valid: std_logic;
  alias S_lstick_x: std_logic_vector(7 downto 0) is R_hid_report(31 downto 24);
  alias S_lstick_y: std_logic_vector(7 downto 0) is R_hid_report(39 downto 32);
  alias S_btn_a: std_logic is R_hid_report(45);
  alias S_btn_b: std_logic is R_hid_report(44);
  alias S_btn_select: std_logic is R_hid_report(54);
  alias S_btn_start: std_logic is R_hid_report(53);
begin

  yes_reg_input: if C_reg_input generate
  process(clk) is
  begin
    if rising_edge(clk) then
      if hid_valid = '1' then
        R_hid_report <= hid_report; -- register to release timing closure
      end if;
      R_hid_valid <= hid_valid;
    end if;
  end process;
  end generate;

  no_reg_input: if not C_reg_input generate
    R_hid_report <= hid_report; -- directly take input
    R_hid_valid <= hid_valid;
  end generate;

  -- hat
  decoded.lstick_x <= S_lstick_x;
  decoded.lstick_y <= S_lstick_y;
  -- simple buttons
  decoded.btn_a <= S_btn_a;
  decoded.btn_b <= S_btn_b;
  decoded.btn_back <= S_btn_select;
  decoded.btn_start <= S_btn_start;
  
end rtl;
